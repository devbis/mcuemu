    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
    .word 0x20001000
    .word reset+1
    .org 0x100
reset:
    movs r0, #7
    bkpt #1
    movs r0, #9
spin:
    b spin
