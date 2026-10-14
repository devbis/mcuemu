    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
@ Thread code on the process stack (CONTROL.SPSEL = 1, PSP = 0x20000800) takes a
@ SysTick every 1000 cycles; the handler runs on the main stack and counts at
@ RAM 0x20000100. The main loop counts at 0x20000104.
    .word 0x20001000
    .word reset+1
    .rept 13
    .word dummy+1
    .endr
    .word systick+1           @ 15: SysTick
    .org 0x100
reset:
    ldr r0, =0x20000800
    msr psp, r0
    movs r0, #2
    msr control, r0           @ thread mode on PSP
    isb
    ldr r0, =0xE000E010
    ldr r1, =999
    str r1, [r0, #4]          @ RVR
    movs r1, #0
    str r1, [r0, #8]          @ CVR
    movs r1, #7
    str r1, [r0]              @ CSR: enable, interrupt, core clock
    ldr r2, =0x20000104
loop:
    ldr r3, [r2]
    adds r3, #1
    str r3, [r2]
    b loop
dummy:
    b dummy
systick:
    push {r4, lr}             @ on the main stack
    ldr r2, =0x20000100
    ldr r3, [r2]
    adds r3, #1
    str r3, [r2]
    mrs r4, msp
    ldr r2, =0x20000108
    str r4, [r2]              @ MSP seen by the handler
    pop {r4, pc}
    .ltorg
