# load-use, RAW chains, store-after-load, jal/jalr, branch on fresh value
main:
  li   gp, data
  lw   t0, 0(gp)        # load-use
  addi t1, t0, 1        # depends on load
  add  t2, t1, t1       # RAW chain
  sw   t2, 4(gp)        # store depends on t2
  lw   t3, 4(gp)        # load what we just stored
  beq  t3, t2, ok       # branch on fresh load
  li   a0, 999
ok:
  jal  ra, f
  mv   a0, t4
  li   a7, 1
  ecall
  li   a7, 10
  ecall
f:
  sub  t4, t3, t0       # (2*(x+1)) - x
  jalr zero, 0(ra)
data: .word 41, 0
