# base: 0x40800000
# Spins forever: for tests that read the timers after a stretch of virtual time.
    .globl _start
_start:
    j     _start
