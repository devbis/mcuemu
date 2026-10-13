    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
@ Cortex-M core: SVC, a nested higher-priority IRQ, FPU context across an IRQ.
@ results (RAM): 0x20000100 isr3 entered, 0x20000104 isr3 resumed after isr4,
@ 0x20000108 isr4, 0x2000010c svc, 0x20000110 main after svc, 0x20000114 s0+s1
    .word 0x20001000
    .word reset+1
    .rept 9
    .word dummy+1
    .endr
    .word svc_h+1             @ 11: SVCall
    .rept 7
    .word dummy+1
    .endr
    .word isr3+1              @ 19: IRQ3
    .word isr4+1              @ 20: IRQ4
    .org 0x100
reset:
    ldr r0, =0xE000ED88
    ldr r1, =0x00F00000
    str r1, [r0]              @ CPACR: enable CP10/CP11
    ldr r0, =0xE000E400
    movs r1, #0x40
    strb r1, [r0, #3]         @ IRQ3 priority 0x40
    movs r1, #0x20
    strb r1, [r0, #4]         @ IRQ4 priority 0x20 (more urgent)
    ldr r0, =0xE000E100
    movs r1, #0x18
    str r1, [r0]              @ enable IRQ3, IRQ4
    vmov.f32 s0, #1.5
    vmov.f32 s1, #2.0
    movs r4, #0x44            @ callee-saved marker
    svc #5
    ldr r2, =0x20000110
    movs r3, #1
    str r3, [r2]
loop:
    vadd.f32 s2, s0, s1
    ldr r2, =0x20000114
    vstr s2, [r2]             @ 3.5 if s0 survived the IRQ, 12.0 if it was clobbered
    b loop
dummy:
    b dummy
svc_h:
    ldr r2, =0x2000010c
    movs r3, #4
    str r3, [r2]
    bx lr
isr3:
    ldr r2, =0x20000100
    movs r3, #1
    str r3, [r2]
    vmov.f32 s0, #10.0        @ clobber FP state: must be restored on return
    ldr r2, =0xE000EF00
    movs r3, #4
    str r3, [r2]              @ STIR: pend IRQ4
    ldr r2, =0x20000104
    movs r3, #2
    str r3, [r2]
    bx lr
isr4:
    ldr r2, =0x20000108
    movs r3, #3
    str r3, [r2]
    bx lr
    .ltorg
