#include "rv32f.h"
#include "../core/isa_registry.h"

#include <fenv.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*
 * RV32F on top of the host's IEEE 754 hardware.
 *
 * The register file holds raw bit patterns (uint32_t). A value only
 * becomes a C `float` for the duration of an arithmetic operation, via
 * memcpy (never a pointer cast: that would break strict aliasing).
 *
 * Arithmetic (add/sub/mul/div/sqrt/fma, int->float) runs on the host FPU
 * with the rounding mode set through <fenv.h>, and the host's exception
 * flags are translated into fcsr.fflags. That gives correctly rounded
 * results and exact flags, including subnormals and underflow (x86 SSE,
 * like RISC-V, detects tininess after rounding).
 *
 * Everything where RISC-V's rules differ from what C/the host does is
 * done by hand on the bits:
 *   - NaN results are always the canonical NaN 0x7fc00000 (x86 would
 *     produce 0xffc00000; C makes no promise at all).
 *   - fmin/fmax: a NaN operand is ignored if the other is a number;
 *     -0 < +0; signalling NaNs raise NV.
 *   - feq is quiet (NV only for signalling NaNs); flt/fle signal NV for
 *     any NaN.
 *   - float->int conversions saturate and raise NV when out of range or
 *     NaN (C's cast is undefined behaviour there).
 *   - fsgnj*, fmv, fclass, flw/fsw never touch a float at all.
 *   - Round-to-nearest-max-magnitude (RMM) has no <fenv.h> equivalent.
 *     It only differs from round-to-nearest-even when the exact result
 *     lies exactly halfway between two floats; see d2f() and fp_arith().
 *   - inf * 0 in a fused multiply-add raises NV even when the addend is a
 *     quiet NaN (IEEE leaves that case to the implementation; RISC-V
 *     requires it).
 *
 * Rounding mode: the instruction's rm field (funct3), or fcsr.frm when
 * rm = 7 (dyn). rm values 5 and 6, or dyn with an invalid frm, make the
 * instruction illegal.
 */

#define OP_LOAD_FP  0x07
#define OP_STORE_FP 0x27
#define OP_FP       0x53
#define OP_FMADD    0x43
#define OP_FMSUB    0x47
#define OP_FNMSUB   0x4B
#define OP_FNMADD   0x4F

#define CANONICAL_NAN 0x7fc00000u
#define SIGN_BIT      0x80000000u

/* ------------------------------------------------------------------ */
/* Helpers you may use (static inline: no warning while unused)        */
/* ------------------------------------------------------------------ */

static inline float f_of(uint32_t bits) { float f; memcpy(&f, &bits, sizeof f); return f; }
static inline uint32_t bits_of(float f) { uint32_t b; memcpy(&b, &f, sizeof b); return b; }

static inline int is_nan_bits(uint32_t b)  { return (b & 0x7f800000u) == 0x7f800000u && (b & 0x007fffffu) != 0; }
static inline int is_snan_bits(uint32_t b) { return is_nan_bits(b) && (b & 0x00400000u) == 0; }

/* ------------------------------------------------------------------ */
/* Rounding mode                                                      */
/* ------------------------------------------------------------------ */

/* The instruction's rm field, or fcsr.frm for dyn. Modes 5/6 (or dyn
 * with an invalid frm) make the instruction illegal: returns -1 having
 * faulted, and the caller must then write nothing at all. */
static int resolve_rm(CPUState *c, const DecodedInstr *i) {
    int rm = (int)i->funct3;
    if (rm == RM_DYN) rm = (int)cpu_frm(c);
    if (rm > RM_RMM) {
        c->status = CPU_ILLEGAL_INSTRUCTION;
        c->fault_addr = c->pc;
        snprintf(c->fault_msg, sizeof(c->fault_msg),
                 "illegal rounding mode %d at pc=0x%08x", rm, c->pc);
        return -1;
    }
    return rm;
}

/* RISC-V rm -> <fenv.h>. RMM has no host mode: run as RNE, fix ties after. */
static int host_mode(int rm) {
    switch (rm) {
    case RM_RTZ: return FE_TOWARDZERO;
    case RM_RDN: return FE_DOWNWARD;
    case RM_RUP: return FE_UPWARD;
    default:     return FE_TONEAREST;                   /* RNE and RMM */
    }
}

/* Host exception flags -> fcsr.fflags bits. */
static uint32_t fflags_of(int fe) {
    uint32_t f = 0;
    if (fe & FE_INVALID)   f |= FFLAG_NV;
    if (fe & FE_DIVBYZERO) f |= FFLAG_DZ;
    if (fe & FE_OVERFLOW)  f |= FFLAG_OF;
    if (fe & FE_UNDERFLOW) f |= FFLAG_UF;
    if (fe & FE_INEXACT)   f |= FFLAG_NX;
    return f;
}

/* RMM fix-up. r is the round-to-nearest-even result of an operation whose
 * exact value is d (exact_d says whether d really is exact). If the exact
 * value lies exactly halfway between r and its neighbour, RNE picked the
 * even one; RMM wants the one with the larger magnitude. */
static float rmm_fixup(float r, double d, int exact_d) {
    if (!exact_d || isnan(r) || isinf(r) || (double)r == d) return r;
    float other = nextafterf(r, (d > (double)r) ? INFINITY : -INFINITY);
    if ((double)r + (double)other != 2.0 * d) return r;   /* not a tie */
    return (fabsf(other) > fabsf(r)) ? other : r;          /* ties away from zero */
}

/* ------------------------------------------------------------------ */
/* Arithmetic on the host FPU                                         */
/* ------------------------------------------------------------------ */

typedef enum { AR_ADD, AR_SUB, AR_MUL, AR_DIV, AR_SQRT, AR_FMA } ArithOp;

static float host_op(ArithOp op, float a, float b, float cc) {
    switch (op) {
    case AR_ADD:  return a + b;
    case AR_SUB:  return a - b;
    case AR_MUL:  return a * b;
    case AR_DIV:  return a / b;
    case AR_SQRT: return sqrtf(a);
    case AR_FMA:  return fmaf(a, b, cc);
    }
    return 0.0f;
}

/* The same operation in double, for RMM tie detection only. */
static double host_op_d(ArithOp op, double a, double b, double cc) {
    switch (op) {
    case AR_ADD:  return a + b;
    case AR_SUB:  return a - b;
    case AR_MUL:  return a * b;
    case AR_DIV:  return a / b;
    case AR_SQRT: return sqrt(a);
    case AR_FMA:  return fma(a, b, cc);
    }
    return 0.0;
}

/* op(A, B, C) on bit patterns, rounded in mode rm. Returns the result
 * bits (any NaN canonicalised) and ORs the exception flags into *flags. */
static uint32_t fp_arith(ArithOp op, uint32_t A, uint32_t B, uint32_t C, int rm, uint32_t *flags) {
    float a = f_of(A), b = f_of(B), cc = f_of(C);
    int nops = (op == AR_SQRT) ? 1 : (op == AR_FMA) ? 3 : 2;

    /* A signalling NaN operand is always invalid (don't trust the host). */
    if (is_snan_bits(A) || (nops >= 2 && is_snan_bits(B)) || (nops == 3 && is_snan_bits(C)))
        *flags |= FFLAG_NV;
    /* RISC-V: inf * 0 in an FMA is invalid even when the addend is a qNaN. */
    if (op == AR_FMA && ((isinf(a) && b == 0.0f) || (a == 0.0f && isinf(b))))
        *flags |= FFLAG_NV;

    int saved = fegetround();
    fesetround(host_mode(rm));
    feclearexcept(FE_ALL_EXCEPT);
    volatile float r = host_op(op, a, b, cc);
    int fe = fetestexcept(FE_ALL_EXCEPT);
    float res = r;

    if (rm == RM_RMM) {
        feclearexcept(FE_ALL_EXCEPT);
        volatile double d = host_op_d(op, (double)a, (double)b, (double)cc);
        int exact_d = !fetestexcept(FE_INEXACT);
        res = rmm_fixup(res, d, exact_d);
    }
    fesetround(saved);
    feclearexcept(FE_ALL_EXCEPT);

    *flags |= fflags_of(fe);
    return isnan(res) ? CANONICAL_NAN : bits_of(res);
}

/* Shared by every arithmetic handler. Resolve the mode FIRST: an illegal
 * instruction must not write a register or a flag. */
static void do_arith(CPUState *c, DecodedInstr *i, ArithOp op, uint32_t A, uint32_t B, uint32_t C) {
    int rm = resolve_rm(c, i);
    if (rm < 0) return;
    uint32_t flags = 0;
    uint32_t r = fp_arith(op, A, B, C, rm, &flags);
    cpu_raise_fflags(c, flags);
    cpu_write_freg(c, (int)i->rd, r);
}

/* ------------------------------------------------------------------ */
/* Conversions                                                        */
/* ------------------------------------------------------------------ */

/* Round a double that holds an exact float value to an integer value,
 * in mode rm. Everything is exact in double, so no flag is lost. */
static double round_to_int(double x, int rm) {
    switch (rm) {
    case RM_RTZ: return trunc(x);
    case RM_RDN: return floor(x);
    case RM_RUP: return ceil(x);
    case RM_RMM: return round(x);                       /* halfway cases away from zero */
    default: {                                          /* RNE: ties to even */
        double f = floor(x), diff = x - f;
        if (diff > 0.5) return f + 1.0;
        if (diff < 0.5) return f;
        return (fmod(f, 2.0) == 0.0) ? f : f + 1.0;
    }
    }
}

/* float -> int32 / uint32: test NaN, round, range-check, saturate. */
static uint32_t cvt_f_to_int(CPUState *c, uint32_t A, int rm, int is_unsigned) {
    uint32_t vmax = is_unsigned ? 0xFFFFFFFFu : 0x7FFFFFFFu;
    uint32_t vmin = is_unsigned ? 0u          : 0x80000000u;
    if (is_nan_bits(A)) { cpu_raise_fflags(c, FFLAG_NV); return vmax; }

    double x = (double)f_of(A);
    double r = round_to_int(x, rm);
    double hi = is_unsigned ? 4294967295.0 : 2147483647.0;
    double lo = is_unsigned ? 0.0          : -2147483648.0;
    if (r > hi) { cpu_raise_fflags(c, FFLAG_NV); return vmax; }   /* clipped: NV only */
    if (r < lo) { cpu_raise_fflags(c, FFLAG_NV); return vmin; }
    if (r != x) cpu_raise_fflags(c, FFLAG_NX);
    if (is_unsigned) return (uint32_t)r;
    return (uint32_t)(int32_t)r;
}

/* int -> float: d is the exact integer value. */
static uint32_t cvt_int_to_f(CPUState *c, double d, int rm) {
    int saved = fegetround();
    fesetround(host_mode(rm));
    feclearexcept(FE_ALL_EXCEPT);
    volatile float r = (float)d;
    int fe = fetestexcept(FE_ALL_EXCEPT);
    float res = r;
    if (rm == RM_RMM) res = rmm_fixup(res, d, 1);
    fesetround(saved);
    feclearexcept(FE_ALL_EXCEPT);
    cpu_raise_fflags(c, fflags_of(fe));
    return bits_of(res);
}

/* ------------------------------------------------------------------ */
/* Handlers                                                           */
/* ------------------------------------------------------------------ */

/* Read an f register inside a handler: FREG(i->rs1). */
#define FREG(i)  cpu_read_freg(c, (int)(i))

static void fp_mem_fault(CPUState *c, uint32_t addr, const char *what) {
    c->status = CPU_MEM_FAULT;
    c->fault_addr = addr;
    snprintf(c->fault_msg, sizeof(c->fault_msg), "%s out of bounds at address 0x%08x", what, addr);
}

/* ---- loads / stores: raw 32-bit moves ---- */

static void exec_flw(CPUState *c, Memory *m, DecodedInstr *i) {
    uint32_t addr = cpu_read_reg(c, (int)i->rs1) + (uint32_t)i->imm;
    uint32_t v;
    if (memory_read_word(m, addr, &v) != MEM_OK) { fp_mem_fault(c, addr, "flw"); return; }
    cpu_write_freg(c, (int)i->rd, v);
}

static void exec_fsw(CPUState *c, Memory *m, DecodedInstr *i) {
    uint32_t addr = cpu_read_reg(c, (int)i->rs1) + (uint32_t)i->imm;
    if (memory_write_word(m, addr, FREG(i->rs2)) != MEM_OK) fp_mem_fault(c, addr, "fsw");
}

/* ---- arithmetic ---- */

static void exec_fadd(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; do_arith(c, i, AR_ADD, FREG(i->rs1), FREG(i->rs2), 0);
}
static void exec_fsub(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; do_arith(c, i, AR_SUB, FREG(i->rs1), FREG(i->rs2), 0);
}
static void exec_fmul(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; do_arith(c, i, AR_MUL, FREG(i->rs1), FREG(i->rs2), 0);
}
static void exec_fdiv(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; do_arith(c, i, AR_DIV, FREG(i->rs1), FREG(i->rs2), 0);
}
static void exec_fsqrt(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; do_arith(c, i, AR_SQRT, FREG(i->rs1), 0, 0);
}

/* Fused multiply-add family: one rounding of (+-rs1 * rs2) +- rs3.
 * Negating = flipping the sign bit, which never signals. */
static void exec_fmadd(CPUState *c, Memory *m, DecodedInstr *i) {      /*  a*b + c */
    (void)m; do_arith(c, i, AR_FMA, FREG(i->rs1), FREG(i->rs2), FREG(i->rs3));
}
static void exec_fmsub(CPUState *c, Memory *m, DecodedInstr *i) {      /*  a*b - c */
    (void)m; do_arith(c, i, AR_FMA, FREG(i->rs1), FREG(i->rs2), FREG(i->rs3) ^ SIGN_BIT);
}
static void exec_fnmsub(CPUState *c, Memory *m, DecodedInstr *i) {     /* -(a*b) + c */
    (void)m; do_arith(c, i, AR_FMA, FREG(i->rs1) ^ SIGN_BIT, FREG(i->rs2), FREG(i->rs3));
}
static void exec_fnmadd(CPUState *c, Memory *m, DecodedInstr *i) {     /* -(a*b) - c */
    (void)m; do_arith(c, i, AR_FMA, FREG(i->rs1) ^ SIGN_BIT, FREG(i->rs2), FREG(i->rs3) ^ SIGN_BIT);
}

/* ---- min / max ---- */

static uint32_t fminmax(CPUState *c, uint32_t A, uint32_t B, int is_max) {
    if (is_snan_bits(A) || is_snan_bits(B)) cpu_raise_fflags(c, FFLAG_NV);   /* 1. always */
    int na = is_nan_bits(A), nb = is_nan_bits(B);
    if (na && nb) return CANONICAL_NAN;                                       /* 2. */
    if (na) return B;                                                         /* 3. NaN never wins */
    if (nb) return A;
    if (((A | B) & ~SIGN_BIT) == 0)                                           /* 4. two zeros */
        return is_max ? (A & B) : (A | B);
    float a = f_of(A), b = f_of(B);                                           /* 5. */
    if (is_max) return (a > b) ? A : B;
    return (a < b) ? A : B;
}

static void exec_fmin(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; cpu_write_freg(c, (int)i->rd, fminmax(c, FREG(i->rs1), FREG(i->rs2), 0));
}
static void exec_fmax(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; cpu_write_freg(c, (int)i->rd, fminmax(c, FREG(i->rs1), FREG(i->rs2), 1));
}

/* ---- sign injection (pure bit operations, no flags, no NaN rules) ---- */

static void exec_fsgnj(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m;
    cpu_write_freg(c, (int)i->rd, (FREG(i->rs1) & ~SIGN_BIT) | (FREG(i->rs2) & SIGN_BIT));
}
static void exec_fsgnjn(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m;
    cpu_write_freg(c, (int)i->rd, (FREG(i->rs1) & ~SIGN_BIT) | (~FREG(i->rs2) & SIGN_BIT));
}
static void exec_fsgnjx(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m;
    cpu_write_freg(c, (int)i->rd, FREG(i->rs1) ^ (FREG(i->rs2) & SIGN_BIT));
}

/* ---- comparisons (result in an x register) ---- */

static void exec_feq(CPUState *c, Memory *m, DecodedInstr *i) {       /* quiet */
    (void)m;
    uint32_t A = FREG(i->rs1), B = FREG(i->rs2);
    if (is_snan_bits(A) || is_snan_bits(B)) cpu_raise_fflags(c, FFLAG_NV);
    int r = !is_nan_bits(A) && !is_nan_bits(B) && f_of(A) == f_of(B);
    cpu_write_reg(c, (int)i->rd, (uint32_t)r);
}
static void exec_flt(CPUState *c, Memory *m, DecodedInstr *i) {       /* signalling */
    (void)m;
    uint32_t A = FREG(i->rs1), B = FREG(i->rs2);
    if (is_nan_bits(A) || is_nan_bits(B)) {
        cpu_raise_fflags(c, FFLAG_NV);
        cpu_write_reg(c, (int)i->rd, 0);
        return;
    }
    cpu_write_reg(c, (int)i->rd, (uint32_t)(f_of(A) < f_of(B)));
}
static void exec_fle(CPUState *c, Memory *m, DecodedInstr *i) {       /* signalling */
    (void)m;
    uint32_t A = FREG(i->rs1), B = FREG(i->rs2);
    if (is_nan_bits(A) || is_nan_bits(B)) {
        cpu_raise_fflags(c, FFLAG_NV);
        cpu_write_reg(c, (int)i->rd, 0);
        return;
    }
    cpu_write_reg(c, (int)i->rd, (uint32_t)(f_of(A) <= f_of(B)));
}

/* ---- conversions ---- */

static void exec_fcvt_w_s(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m;
    int rm = resolve_rm(c, i);
    if (rm < 0) return;
    cpu_write_reg(c, (int)i->rd, cvt_f_to_int(c, FREG(i->rs1), rm, 0));
}
static void exec_fcvt_wu_s(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m;
    int rm = resolve_rm(c, i);
    if (rm < 0) return;
    cpu_write_reg(c, (int)i->rd, cvt_f_to_int(c, FREG(i->rs1), rm, 1));
}

static void exec_fcvt_s_w(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m;
    int rm = resolve_rm(c, i);
    if (rm < 0) return;
    double d = (double)(int32_t)cpu_read_reg(c, (int)i->rs1);
    cpu_write_freg(c, (int)i->rd, cvt_int_to_f(c, d, rm));
}
static void exec_fcvt_s_wu(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m;
    int rm = resolve_rm(c, i);
    if (rm < 0) return;
    double d = (double)cpu_read_reg(c, (int)i->rs1);
    cpu_write_freg(c, (int)i->rd, cvt_int_to_f(c, d, rm));
}

/* ---- moves between register files (bits unchanged) ---- */

static void exec_fmv_x_w(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; cpu_write_reg(c, (int)i->rd, FREG(i->rs1));
}
static void exec_fmv_w_x(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m; cpu_write_freg(c, (int)i->rd, cpu_read_reg(c, (int)i->rs1));
}

/* ---- classify: exactly one bit set ---- */

static void exec_fclass(CPUState *c, Memory *m, DecodedInstr *i) {
    (void)m;
    uint32_t b = FREG(i->rs1);
    int neg = (b & SIGN_BIT) != 0;
    uint32_t exp = (b >> 23) & 0xFF, frac = b & 0x007FFFFFu;
    int cls;
    if (exp == 0xFF)   cls = frac ? ((frac & 0x00400000u) ? 9 : 8)   /* qNaN : sNaN */
                              : (neg ? 0 : 7);                        /* -inf : +inf */
    else if (exp == 0) cls = frac ? (neg ? 2 : 5) : (neg ? 3 : 4);    /* subnormal : zero */
    else               cls = neg ? 1 : 6;                             /* normal */
    cpu_write_reg(c, (int)i->rd, 1u << cls);
}

/* ------------------------------------------------------------------ */
/* Registration                                                       */
/* ------------------------------------------------------------------ */

#define FFF (IF_RD_FP | IF_RS1_FP | IF_RS2_FP)   /* fd, fs1, fs2 */

static void reg(const char *mn, uint32_t opcode, int32_t f3, int32_t f7, InstrFormat fmt,
                ExecuteFn fn, uint32_t flags, uint32_t rs2_sel) {
    register_instruction_ex((InstructionHandler){mn, opcode, f3, f7, fmt, fn}, flags, rs2_sel);
}

void register_rv32f(void) {
    reg("flw",       OP_LOAD_FP,  0x2, -1,   FMT_I_LOAD, exec_flw,  IF_RD_FP,  0);
    reg("fsw",       OP_STORE_FP, 0x2, -1,   FMT_S,      exec_fsw,  IF_RS2_FP, 0);

    reg("fadd.s",    OP_FP, -1, 0x00, FMT_R,  exec_fadd,  FFF | IF_HAS_RM, 0);
    reg("fsub.s",    OP_FP, -1, 0x04, FMT_R,  exec_fsub,  FFF | IF_HAS_RM, 0);
    reg("fmul.s",    OP_FP, -1, 0x08, FMT_R,  exec_fmul,  FFF | IF_HAS_RM, 0);
    reg("fdiv.s",    OP_FP, -1, 0x0C, FMT_R,  exec_fdiv,  FFF | IF_HAS_RM, 0);
    reg("fsqrt.s",   OP_FP, -1, 0x2C, FMT_R2, exec_fsqrt, IF_RD_FP | IF_RS1_FP | IF_HAS_RM | IF_RS2_FIXED, 0);

    reg("fsgnj.s",   OP_FP, 0x0, 0x10, FMT_R, exec_fsgnj,  FFF, 0);
    reg("fsgnjn.s",  OP_FP, 0x1, 0x10, FMT_R, exec_fsgnjn, FFF, 0);
    reg("fsgnjx.s",  OP_FP, 0x2, 0x10, FMT_R, exec_fsgnjx, FFF, 0);
    reg("fmin.s",    OP_FP, 0x0, 0x14, FMT_R, exec_fmin,   FFF, 0);
    reg("fmax.s",    OP_FP, 0x1, 0x14, FMT_R, exec_fmax,   FFF, 0);

    reg("feq.s",     OP_FP, 0x2, 0x50, FMT_R, exec_feq, IF_RS1_FP | IF_RS2_FP, 0);
    reg("flt.s",     OP_FP, 0x1, 0x50, FMT_R, exec_flt, IF_RS1_FP | IF_RS2_FP, 0);
    reg("fle.s",     OP_FP, 0x0, 0x50, FMT_R, exec_fle, IF_RS1_FP | IF_RS2_FP, 0);

    reg("fcvt.w.s",  OP_FP, -1, 0x60, FMT_R2, exec_fcvt_w_s,  IF_RS1_FP | IF_HAS_RM | IF_RS2_FIXED, 0);
    reg("fcvt.wu.s", OP_FP, -1, 0x60, FMT_R2, exec_fcvt_wu_s, IF_RS1_FP | IF_HAS_RM | IF_RS2_FIXED, 1);
    reg("fcvt.s.w",  OP_FP, -1, 0x68, FMT_R2, exec_fcvt_s_w,  IF_RD_FP  | IF_HAS_RM | IF_RS2_FIXED, 0);
    reg("fcvt.s.wu", OP_FP, -1, 0x68, FMT_R2, exec_fcvt_s_wu, IF_RD_FP  | IF_HAS_RM | IF_RS2_FIXED, 1);

    reg("fmv.x.w",   OP_FP, 0x0, 0x70, FMT_R2, exec_fmv_x_w, IF_RS1_FP | IF_RS2_FIXED, 0);
    reg("fclass.s",  OP_FP, 0x1, 0x70, FMT_R2, exec_fclass,  IF_RS1_FP | IF_RS2_FIXED, 0);
    reg("fmv.w.x",   OP_FP, 0x0, 0x78, FMT_R2, exec_fmv_w_x, IF_RD_FP  | IF_RS2_FIXED, 0);
    /* Older spellings of the two moves (accepted by RARS too). Registered
     * after the official names, so decoding finds those first. */
    reg("fmv.x.s",   OP_FP, 0x0, 0x70, FMT_R2, exec_fmv_x_w, IF_RS1_FP | IF_RS2_FIXED, 0);
    reg("fmv.s.x",   OP_FP, 0x0, 0x78, FMT_R2, exec_fmv_w_x, IF_RD_FP  | IF_RS2_FIXED, 0);

    /* R4: funct7 holds rs3 and the 2-bit format field (00 = single), so
     * it is "don't care" for the registry. */
    reg("fmadd.s",   OP_FMADD,  -1, -1, FMT_R4, exec_fmadd,  FFF | IF_RS3_FP | IF_HAS_RM, 0);
    reg("fmsub.s",   OP_FMSUB,  -1, -1, FMT_R4, exec_fmsub,  FFF | IF_RS3_FP | IF_HAS_RM, 0);
    reg("fnmsub.s",  OP_FNMSUB, -1, -1, FMT_R4, exec_fnmsub, FFF | IF_RS3_FP | IF_HAS_RM, 0);
    reg("fnmadd.s",  OP_FNMADD, -1, -1, FMT_R4, exec_fnmadd, FFF | IF_RS3_FP | IF_HAS_RM, 0);
}
