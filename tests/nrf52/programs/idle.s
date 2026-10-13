    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
    .word 0x20040000          @ initial SP
    .word reset+1
    .rept 62
    .word idle+1
    .endr
    .org 0x100
reset:
idle:
    wfi                       @ sleep: the core's clock jumps to the next event
    b idle
