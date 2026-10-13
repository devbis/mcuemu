    .syntax unified
    .thumb
    .cpu cortex-m4
    .section .text
@ A store to non-writable memory (flash) at 0x8000.
    .word 0x20001000
    .word reset+1
    .org 0x100
reset:
    ldr r0, =0x00008000
    ldr r1, =0x12345678
    str r1, [r0]
spin:
    b spin
    .ltorg
