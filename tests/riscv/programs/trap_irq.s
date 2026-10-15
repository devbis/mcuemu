# ecall goes to the guest's trap handler (which skips it), then the hart sleeps
# in wfi until the host raises an interrupt with uc_riscv_irq(); the handler
# acknowledges the peripheral (a store to 0x60000008, which drops the level) and
# records mcause in s1; the program then stops at ebreak.
    .option rvc
    .globl _start
_start:
    la    t0, handler
    csrw  mtvec, t0
    li    s0, 0               # ecall traps taken
    li    s1, 0               # mcause of the interrupt
    ecall
    csrsi mstatus, 8          # MIE
wait:
    wfi
    beqz  s1, wait
    ebreak
    .balign 4
handler:
    csrr  t2, mcause
    bltz  t2, irq
    csrr  t3, mepc
    addi  t3, t3, 4
    csrw  mepc, t3
    addi  s0, s0, 1
    mret
irq:
    li    t4, 0x60000008
    sw    zero, 0(t4)         # ack: the controller deasserts the line
    mv    s1, t2
    mret
