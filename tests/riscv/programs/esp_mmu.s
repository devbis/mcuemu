# base: 0x40800000
# Maps flash page 2 at cache page 3 through the SPI0 MMU table (index 3, content
# 0x200 | 2), then reads the first word of 0x42030000 into s1 and stops at ebreak.
    .globl _start
_start:
    li    t0, 0x60002000
    li    t1, 3
    sw    t1, 0x380(t0)        # MMU_ITEM_INDEX
    li    t1, 0x202
    sw    t1, 0x37c(t0)        # MMU_ITEM_CONTENT: valid, flash page 2
    li    t2, 0x42030000
    lw    s1, 0(t2)
    lw    s2, 0x37c(t0)        # reads back the entry
    ebreak
