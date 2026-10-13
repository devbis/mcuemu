    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
@ WFE sleep: IRQ0 wakes the core; r4 counts wake-ups, RAM 0x20000100 counts ISR runs.
    .word 0x20001000
    .word reset+1
    .rept 14
    .word dummy+1
    .endr
    .word isr0+1              @ 16: IRQ0
    .org 0x100
reset:
    ldr r0, =0xE000E100
    movs r1, #1
    str r1, [r0]              @ enable IRQ0
    movs r4, #0
again:
    wfe
    adds r4, #1
    b again
dummy:
    b dummy
isr0:
    ldr r2, =0x20000100
    ldr r3, [r2]
    adds r3, #1
    str r3, [r2]
    bx lr
    .ltorg
