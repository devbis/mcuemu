# mtvec in vectored mode: interrupt N enters at base + 4*N. Each slot stores its
# number in s2 and returns. Execution stops at ebreak once s2 is set.
    .globl _start
_start:
    la    t0, table
    ori   t0, t0, 1
    csrw  mtvec, t0
    li    s2, 0
    csrsi mstatus, 8
wait:
    wfi
    beqz  s2, wait
    ebreak
    .balign 256
table:
    .option push
    .option norvc
    .rept 32
    j     slot
    .endr
    .option pop
slot:
    li    t4, 0x60000008
    sw    zero, 0(t4)         # ack
    csrr  s2, mcause
    andi  s2, s2, 0x1f
    mret
