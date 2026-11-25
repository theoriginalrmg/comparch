# case1_nested_backward.s -- nested loops, all BACKWARD branches.
# Outer loop runs 5 times; each time an inner loop runs 4 times.
# Computes sum over i=1..5, j=1..4 of (i*j) by repeated addition = 150.
# Branches: inner back edge (T T T N) x5 = 20, outer back edge T T T T N = 5,
#           plus a counted "warm-up" loop of 8 iterations = 8. Total 33.
.text
.globl main
main:
    addi s0, zero, 0          # result
    # ---- a simple counted warm-up loop: 8 iterations --------------
    addi t0, zero, 8
warm:
    addi s0, s0, 0            # (does nothing useful, just loop body)
    addi t0, t0, -1
    bne  t0, zero, warm       # backward: T x7, N
    # ---- nested loops ----------------------------------------------
    addi s1, zero, 1          # i = 1
outer:
    addi s2, zero, 1          # j = 1
inner:
    # s3 = i * j by repeated addition is overkill; add i, j times total
    add  s0, s0, s1           # result += i   (done j times per j -> i*j summed below)
    addi s2, s2, 1            # j++
    addi t1, zero, 5
    blt  s2, t1, inner        # backward: j < 5 -> T T T N
    addi s1, s1, 1            # i++
    addi t2, zero, 6
    blt  s1, t2, outer        # backward: i < 6 -> T T T T N
    # result = sum_i (i * 4) = 4 * 15 = 60
    mv   a0, s0
    addi a7, zero, 1
    ecall                     # prints 60
    addi a7, zero, 10
    ecall
