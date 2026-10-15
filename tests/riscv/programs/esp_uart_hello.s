# base: 0x40800000
# Boot image payload for the ESP32-C6 model: writes "Hi!\n" to the UART0 FIFO
# (0x60000000), then calls the ROM: memcpy (0x400004ac) of the four bytes to the
# SRAM, __udivdi3 (0x400009cc) of 1000000000000 / 1000, and prints the quotient's
# low byte, and stops at ebreak.
    .globl _start
_start:
    li    t0, 0x60000000
    li    t1, 'H'
    sw    t1, 0(t0)
    li    t1, 'i'
    sw    t1, 0(t0)
    li    t1, '!'
    sw    t1, 0(t0)
    li    t1, '\n'
    sw    t1, 0(t0)

    la    a1, text
    li    a0, 0x40810000
    li    a2, 4
    li    t2, 0x400004ac       # memcpy(dst, src, len)
    jalr  t2
    li    t0, 0x40810000
    lw    s1, 0(t0)            # the copied "Hi!\n"

    li    a0, 0xd4a51000       # 1000000000000 = 0xE8_D4A5_1000
    li    a1, 0xe8
    li    a2, 1000
    li    a3, 0
    li    t2, 0x400009cc       # __udivdi3
    jalr  t2
    mv    s2, a0               # 1000000000
    ebreak
    .balign 4
text:
    .ascii "Hi!\n"
