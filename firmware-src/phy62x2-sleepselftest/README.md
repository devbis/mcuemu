# PHY62x2 sleep self test

A Zephyr application for the PB-03F (`pb_03f`) that exercises the sleeps of the PHY62x2 and prints the
result with `printk` on UART0: three suspends to RAM of 500 ms and then a deep sleep of 500 ms that keeps a
`zephyr,retained-ram` counter. It runs on the board and in the emulator
(`examples/phy6252-sleepselftest.yaml`, `tests/firmware/test_phy62xx_sleep.py`).

Build it with the Zephyr tree of the PHY62x2 port (16 MHz core, no MCUboot; the code is XIP at
`0x11010100`) and keep the package `zephyr.phy62x2.bin` as `firmware/phy62x2-sleepselftest.bin`:

    west build -b pb_03f firmware-src/phy62x2-sleepselftest -- \
        -DDTC_OVERLAY_FILE=$PWD/firmware-src/phy62x2-sleepselftest/app.overlay

`-DNO_RC=1` builds the variant for the negative test (`firmware/phy62x2-sleepselftest-norc.bin`): it clears
the RC32K wake-up bit of `AON->PMCTL0` before the first sleep, and the chip must not wake.
