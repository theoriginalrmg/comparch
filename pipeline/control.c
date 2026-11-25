#include "control.h"
#include <stdio.h>
#include <string.h>

/* x0 is hardwired to zero, so reading it never depends on an older
 * instruction and writing it never produces a value anyone can consume.
 * Mapping it to REG_NONE here means hazard logic never has to special-case it. */
static inline int reg_or_none(uint32_t r) {
    return (r == 0) ? REG_NONE : (int)r;
}

/* Operand in the x or f register file depending on the handler's flag. */
static inline int operand(uint32_t r, uint32_t flags, uint32_t fp_flag) {
    if (flags & fp_flag) return REG_FP_BASE + (int)(r & 0x1F);
    return reg_or_none(r);
}

uint32_t unified_read(const CPUState *cpu, int r) {
    return REG_IS_FP(r) ? cpu_read_freg(cpu, REG_INDEX(r)) : cpu_read_reg(cpu, r);
}

void unified_write(CPUState *cpu, int r, uint32_t v) {
    if (REG_IS_FP(r)) cpu_write_freg(cpu, REG_INDEX(r), v);
    else              cpu_write_reg(cpu, r, v);
}

const char *unified_name(int r) {
    static char buf[8];
    snprintf(buf, sizeof(buf), "%c%d", REG_IS_FP(r) ? 'f' : 'x', REG_INDEX(r));
    return buf;
}

/* RISC-V load/store funct3: bits [1:0] = log2(size), bit 2 = unsigned (loads). */
static inline void set_mem_width(Control *c, uint32_t funct3) {
    c->mem_size = 1 << (funct3 & 0x3);
    c->mem_unsigned = (funct3 & 0x4) ? 1 : 0;
}

void control_from_instr(const InstructionHandler *h, const DecodedInstr *d, Control *out) {
    memset(out, 0, sizeof(*out));
    out->src1 = out->src2 = out->src3 = out->dst = REG_NONE;
    uint32_t fl = isa_flags(h);

    switch (h->format) {
    case FMT_R:                                   /* rd, rs1, rs2 */
        out->src1 = operand(d->rs1, fl, IF_RS1_FP);
        out->src2 = operand(d->rs2, fl, IF_RS2_FP);
        out->dst  = operand(d->rd,  fl, IF_RD_FP);
        break;

    case FMT_I:                                   /* rd, rs1, imm */
    case FMT_I_SHAMT:                             /* rd, rs1, shamt */
        out->src1 = operand(d->rs1, fl, IF_RS1_FP);
        out->dst  = operand(d->rd,  fl, IF_RD_FP);
        break;

    case FMT_I_LOAD:                              /* rd, imm(rs1): the address is always an x register */
        out->src1   = reg_or_none(d->rs1);
        out->dst    = operand(d->rd, fl, IF_RD_FP);
        out->mem_op = MEMOP_LOAD;
        set_mem_width(out, d->funct3);
        break;

    case FMT_S:                                   /* rs2, imm(rs1): rs2 is the data */
        out->src1   = reg_or_none(d->rs1);
        out->src2   = operand(d->rs2, fl, IF_RS2_FP);
        out->mem_op = MEMOP_STORE;
        set_mem_width(out, d->funct3);
        break;

    case FMT_B:                                   /* rs1, rs2, label */
        out->src1      = reg_or_none(d->rs1);
        out->src2      = reg_or_none(d->rs2);
        out->is_branch = 1;
        break;

    case FMT_U:                                   /* rd, imm */
        out->dst = reg_or_none(d->rd);
        break;

    case FMT_J:                                   /* jal rd, label: writes the link value */
        out->dst     = reg_or_none(d->rd);
        out->is_jump = 1;
        break;

    case FMT_I_JALR:                              /* jalr rd, imm(rs1) */
        out->src1    = reg_or_none(d->rs1);
        out->dst     = reg_or_none(d->rd);
        out->is_jump = 1;
        break;

    case FMT_I_SYS:                               /* ecall: runs in EX, reads a7 (service) and a0 (argument) */
        out->src1     = 17;                       /* a7 */
        out->src2     = 10;                       /* a0 */
        out->is_ecall = 1;
        break;

    case FMT_R2:                                  /* rd, rs1: the rs2 field is part of the opcode */
        out->src1 = operand(d->rs1, fl, IF_RS1_FP);
        out->dst  = operand(d->rd,  fl, IF_RD_FP);
        break;

    case FMT_R4:                                  /* fused multiply-add: rd, rs1, rs2, rs3 */
        out->src1 = operand(d->rs1, fl, IF_RS1_FP);
        out->src2 = operand(d->rs2, fl, IF_RS2_FP);
        out->src3 = operand(d->rs3, fl, IF_RS3_FP);
        out->dst  = operand(d->rd,  fl, IF_RD_FP);
        break;

    case FMT_CSR:                                 /* rd, csr, rs1 (x registers; fcsr itself is
                                                     read/written in EX, in order) */
        out->src1 = reg_or_none(d->rs1);
        out->dst  = reg_or_none(d->rd);
        break;

    case FMT_CSRI:                                /* rd, csr, uimm5: the rs1 field is an immediate */
        out->dst = reg_or_none(d->rd);
        break;
    }
}
