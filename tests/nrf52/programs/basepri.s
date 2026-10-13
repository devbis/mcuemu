    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
@ BASEPRI: IRQ3 (priority 0x40) is blocked while BASEPRI is 0x30; main lifts
@ BASEPRI when the host sets the flag at RAM 0x20000200. RAM 0x20000100 counts ISR runs.
    .word 0x20001000
    .word reset+1
    .rept 17
    .word dummy+1
    .endr
    .word isr3+1              @ 19: IRQ3
    .org 0x100
reset:
    ldr r0, =0xE000E400
    movs r1, #0x40
    strb r1, [r0, #3]
    ldr r0, =0xE000E100
    movs r1, #8
    str r1, [r0]              @ enable IRQ3
    movs r0, #0x30
    msr basepri, r0
    ldr r2, =0x20000200
wait:
    ldr r1, [r2]
    cmp r1, #0
    beq wait
    movs r0, #0
    msr basepri, r0
done:
    b done
dummy:
    b dummy
isr3:
    ldr r2, =0x20000100
    ldr r3, [r2]
    adds r3, #1
    str r3, [r2]
    bx lr
    .ltorg
