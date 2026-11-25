# case2_many_branches.s -- one backward loop with several branches inside.
# Loops i = 0..15 over an array of 16 words and, for each element:
#   - counts negatives          (forward branch, data dependent)
#   - counts values > 10        (forward branch, data dependent)
#   - counts even indices       (forward branch, alternates T N T N ...)
# The loop back edge is a backward branch taken 15 times then not taken.
.data
arr: .word 5, -3, 12, 7, -8, 20, 0, 15, -1, 11, 4, -6, 30, 2, -9, 13
.text
.globl main
main:
    la   s0, arr              # base address
    addi s1, zero, 0          # i
    addi s2, zero, 16         # n
    addi s3, zero, 0          # negatives
    addi s4, zero, 0          # big (> 10)
    addi s5, zero, 0          # even indices
    addi s6, zero, 10
loop:
    slli t0, s1, 2
    add  t0, s0, t0
    lw   t1, 0(t0)            # load ...
    bge  t1, zero, notneg     # ... used immediately (load-use); forward
    addi s3, s3, 1
notneg:
    bge  s6, t1, notbig       # forward: skip if t1 <= 10
    addi s4, s4, 1
notbig:
    andi t2, s1, 1
    bne  t2, zero, odd        # forward, alternates: N T N T ...
    addi s5, s5, 1
odd:
    addi s1, s1, 1
    blt  s1, s2, loop         # BACKWARD loop edge: T x15, N
    # negatives = 5 (-3,-8,-1,-6,-9), big = 6 (12,20,15,11,30,13), evens = 8 -> prints 19
    add  a0, s3, s4
    add  a0, a0, s5           # printed: negatives + big + evens
    addi a7, zero, 1
    ecall
    addi a7, zero, 10
    ecall
