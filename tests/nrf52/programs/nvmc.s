    .syntax unified
    .thumb
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .section .text
@ nRF52840 NVMC: program 0xA5A5A5A5 then 0x0F0F0F0F into the same flash word at
@ 0x80000 (the result is the AND), then try a write with writes disabled.
    .word 0x20040000
    .word reset+1
    .rept 62
    .word dummy+1
    .endr
    .org 0x100
reset:
    ldr r0, =0x4001E504       @ NVMC.CONFIG
    movs r1, #1               @ WEN
    str r1, [r0]
    ldr r2, =0x00080000
    ldr r3, =0xA5A5A5A5
    str r3, [r2]
    ldr r3, =0x0F0F0F0F
    str r3, [r2]
    movs r1, #0               @ read-only again
    str r1, [r0]
    ldr r3, =0x00000000
    str r3, [r2]              @ ignored
    ldr r2, =0x10001080       @ UICR word, written with WEN too
    movs r1, #1
    str r1, [r0]
    ldr r3, =0x12345678
    str r3, [r2]
    movs r1, #0
    str r1, [r0]
spin:
    b spin
dummy:
    b dummy
    .ltorg
