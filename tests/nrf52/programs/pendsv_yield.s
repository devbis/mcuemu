    .syntax unified
    .thumb
    .cpu cortex-m4
    .section .text
@ A PendSV yield must be taken before the thread executes the instruction after ICSR.PENDSVSET.
    .word 0x20001000
    .word reset+1
    .rept 12
    .word dummy+1
    .endr
    .word pendsv+1
    .org 0x100
reset:
    ldr r0, =0x20000100
    movs r1, #1
    str r1, [r0]
    ldr r0, =0xe000ed04
    ldr r1, =0x10000000
    str r1, [r0]                 @ pend PendSV
    ldr r0, =0x20000100
    movs r1, #2
    str r1, [r0]                 @ must run only after PendSV returns
loop:
    b loop
dummy:
    b dummy
pendsv:
    ldr r0, =0x20000100
    ldr r1, [r0]
    ldr r0, =0x20000104
    str r1, [r0]                 @ capture state at exception entry
    bx lr
    .ltorg
