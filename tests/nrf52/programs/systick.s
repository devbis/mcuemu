    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
@ SysTick every 1000 cycles; the handler counts at RAM 0x20000100.
    .word 0x20001000
    .word reset+1
    .rept 13
    .word dummy+1
    .endr
    .word systick+1           @ 15: SysTick
    .org 0x100
reset:
    ldr r0, =0xE000E010
    ldr r1, =999
    str r1, [r0, #4]          @ RVR
    movs r1, #0
    str r1, [r0, #8]          @ CVR
    movs r1, #7
    str r1, [r0]              @ CSR: enable, interrupt, core clock
loop:
    b loop
dummy:
    b dummy
systick:
    ldr r2, =0x20000100
    ldr r3, [r2]
    adds r3, #1
    str r3, [r2]
    bx lr
    .ltorg
