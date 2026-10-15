# base: 0x40800000
# Like esp_irq.s, but the wfi runs with mstatus.MIE cleared (esp-hal's idle loop).
# Interrupt path of the ESP32-C6 model: SYSTIMER target 0 (source 57) routed
# through the interrupt matrix to CPU line 5, enabled and prioritised in the PLIC,
# alarm at 1600 ticks (100 us), vectored mtvec. The hart sleeps in wfi until the
# alarm; the handler acknowledges the timer, counts in s2 and keeps mcause in s3.
    .globl _start
_start:
    la    t0, table
    ori   t0, t0, 1
    csrw  mtvec, t0
    li    s2, 0
    li    s3, 0

    li    t0, 0x600100e4       # INTMTX: source 57 -> CPU line 5
    li    t1, 5
    sw    t1, 0(t0)
    li    t0, 0x20001000       # PLIC_MX
    li    t1, 1 << 5
    sw    t1, 0(t0)            # enable line 5
    li    t1, 3
    sw    t1, 0x24(t0)         # priority of line 5

    li    t0, 0x6000a000       # SYSTIMER
    sw    zero, 0x1c(t0)       # TARGET0_HI
    li    t1, 1600
    sw    t1, 0x20(t0)         # TARGET0_LO
    lw    t1, 0(t0)
    li    t2, 1 << 24
    or    t1, t1, t2
    sw    t1, 0(t0)            # CONF.TARGET0_WORK_EN
    li    t1, 1
    sw    t1, 0x50(t0)         # COMP0_LOAD
    sw    t1, 0x64(t0)         # INT_ENA
    csrsi mstatus, 8
wait:
    csrrci a0, mstatus, 8      # embassy-style idle: interrupts off around the wfi,
    wfi                        # which wakes on the pending line without taking it,
    csrs  mstatus, a0          # and the handler runs when MIE comes back
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
    csrr  s3, mcause
    li    t4, 0x6000a000
    li    t5, 1
    sw    t5, 0x6c(t4)         # INT_CLR: the source drops
    addi  s2, s2, 1
    mret
