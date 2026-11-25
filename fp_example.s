# fp_example.s -- a small floating-point program, modelled on the Lab 5
# sheet's t4: a dot product, Newton's method for sqrt(2), a Horner
# polynomial, and a few flag-raising operations.
# Prints three integers back to back (each result x 1000): 120000 1414 1000
# -> "12000014141000", the same output the Lab 5 sheet gives for t4.
.data
va:    .float 1.0, 2.0, 3.0, 4.0
vb:    .float 4.0, 5.0, 6.0, 7.0     # dot = 4+10+18+28 = 60 -> x2 below = 120
half:  .float 0.5
two:   .float 2.0
thou:  .float 1000.0
coef:  .float 2.0, -3.0, 1.0, 1.0    # p(x) = 2x^3 - 3x^2 + x + 1, p(1) = 1
.text
.globl main
main:
# ---- 1. dot product: flw -> fmadd chain (load-use + FP forwarding) ----
    la   s0, va
    la   s1, vb
    addi t0, zero, 4          # n
    fmv.w.x fs0, zero         # acc = 0.0
dot:
    flw  ft0, 0(s0)
    flw  ft1, 0(s1)           # ft1 used right away below -> load-use stall
    fmadd.s fs0, ft0, ft1, fs0   # acc += a*b  (3 FP sources: fwd3 on fs0)
    addi s0, s0, 4
    addi s1, s1, 4
    addi t0, t0, -1
    bne  t0, zero, dot        # backward loop branch
    fadd.s fs0, fs0, fs0      # 60 + 60 = 120
# ---- 2. Newton: x = 0.5 * (x + 2/x), 4 iterations from x = 2 ----
    la   t1, half
    flw  fa1, 0(t1)           # 0.5
    la   t1, two
    flw  fa2, 0(t1)           # 2.0
    fmv.s fs1, fa2            # x = 2.0
    addi t0, zero, 4
newton:
    fdiv.s ft2, fa2, fs1      # 2/x
    fadd.s ft2, fs1, ft2      # x + 2/x
    fmul.s fs1, ft2, fa1      # x = 0.5 * (...)   -> converges to 1.4142135
    addi t0, t0, -1
    bne  t0, zero, newton
# ---- 3. Horner: p(1) = ((2*1 - 3)*1 + 1)*1 + 1 = 1 ----
    la   t1, coef
    flw  fs2, 0(t1)           # acc = 2
    addi t2, zero, 1
    fcvt.s.w ft3, t2          # x = 1.0  (int -> float)
    flw  ft4, 4(t1)
    fmadd.s fs2, fs2, ft3, ft4   # acc = acc*x + c1
    flw  ft4, 8(t1)
    fmadd.s fs2, fs2, ft3, ft4
    flw  ft4, 12(t1)
    fmadd.s fs2, fs2, ft3, ft4   # = 1.0
# ---- 4. scale by 1000, convert to int, print 120000 1414 1000 ----
    la   t1, thou
    flw  fa3, 0(t1)
    fmul.s ft5, fs0, fa3
    fcvt.w.s a0, ft5, rtz     # 120 * 1000 = 120000
    addi a7, zero, 1
    ecall
    fmul.s ft5, fs1, fa3
    fcvt.w.s a0, ft5, rtz     # 1414
    addi a7, zero, 1
    ecall
    fmul.s ft5, fs2, fa3
    fcvt.w.s a0, ft5, rtz     # 1000
    addi a7, zero, 1
    ecall
# ---- 5. a few flags: compare, min, and an inexact division ----
    flt.s s2, fs1, fa2        # 1.414 < 2.0 -> s2 = 1
    fmin.s fs3, fs1, fa2      # 1.414...
    fdiv.s fs4, fa2, fs1      # inexact -> NX stays set in fcsr
    addi a7, zero, 10
    ecall
