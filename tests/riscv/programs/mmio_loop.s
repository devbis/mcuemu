# Stores a counter to the peripheral window at 0x60000000 and reads a register
# back, forever: for the clock, deadline and MMIO paths of riscv::Core.
    .globl _start
_start:
    li    t0, 0x60000000
    li    t1, 0
loop:
    addi  t1, t1, 1
    sw    t1, 0(t0)
    lw    t2, 4(t0)
    j     loop
