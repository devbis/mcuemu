    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
@ AIRCR.SYSRESETREQ.
    .word 0x20001000
    .word reset+1
    .org 0x100
reset:
    ldr r0, =0xE000ED0C
    ldr r1, =0x05FA0004
    str r1, [r0]
spin:
    b spin
    .ltorg
