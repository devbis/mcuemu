/* SPDX-License-Identifier: MIT */
/* CPU2 image: flash at 0x080E0000 (install with offset 0xE0000), private RAM in
 * SRAM2b so it stays clear of the mailbox (SRAM2a) and of the CPU1 image. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x080E0000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x20038000, LENGTH = 32K
}
ENTRY(reset_handler)
SECTIONS
{
    .text : { KEEP(*(.vectors)) *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
    _sidata = LOADADDR(.data);
    .data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
    .bss  : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
    _estack = ORIGIN(RAM) + LENGTH(RAM);
    /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}
