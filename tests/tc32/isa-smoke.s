.syntax unified
.text
.globl _start
_start:
    tmov r0, #5
    tadd r0, #2
    tcmp r0, #7
    tadd r4, r0, #3
    tsub r5, r4, #2
    tmov r6, r5
    tjeq .Lgood
    tmov r2, #0xee
.Lgood:
    tjl .Lfunc
    tmov r3, #1
    .short 0xcf00
.Lfunc:
    tmov r2, #42
    tjex lr
