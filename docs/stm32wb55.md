# STM32WB55

`stm32wb55` models the STM32WB55's two application cores in `sim`: CPU1 uses
the shared Cortex-M4 core at 64 MHz, and CPU2 uses a separate Cortex-M0 core at
32 MHz. The two cores have independent CPU state and cycle clocks, and map the
same flash and SRAM storage. CPU1 starts CPU2 by setting C2BOOT. CPU2's vector
table is found in its configured raw image, or in the built-in model ROM when
the modeled Thread RCP is selected.
While CPU2 is running, CPU1 yields at one-microsecond service intervals so the
two instruction streams observe shared SRAM and IPCC changes on a common virtual
timeline. IPCC changes request an earlier service.

## CPU2 image

Provide the CPU2 image separately from the Zephyr CPU1 image:

```yaml
devices:
  - id: wb55
    mcus:
      - id: main
        model: stm32wb55
        firmware: ../firmware/stm32wb55-cpu1.bin
        cpu2_firmware:
          - file: ../firmware/stm32wb55-cpu2.bin
            offset: 0xC0000 # use the CPU2 image's linked flash address
```

`cpu2_firmware` accepts the same `file`, `offset`, `from`, and `size` fields as
`firmware`. The configured bytes are mapped into the chip's shared flash, and
CPU2 begins executing when CPU1 enables C2BOOT. Choose an offset that matches
the binary's link address. The emulator expects executable bytes with a Cortex-M
vector table and rejects images without one. It does not unpack encrypted STM32
wireless-firmware packages distributed for the Firmware Upgrade Service (FUS);
those packages do not contain directly executable CPU2 bytes. ST documents the
wireless-stack images as encrypted and signed for FUS in
[AN5185](https://www.st.com/resource/en/application_note/dm00513965-st-firmware-upgrade-services-for-stm32wb-series-stmicroelectronics.pdf).

CPU2 runs on a separate Cortex-M0 core profile at 32 MHz, with independent CPU
state and a cycle clock. It shares flash and SRAM with CPU1 and accesses the
same IPCC status and masks. While both cores are active, the simulator
schedules CPU2 at one-microsecond intervals against the same virtual timeline;
IPCC updates can request an earlier service. Different raw CPU2 firmware images
can be loaded per simulation and execute as M0 instructions. A synthetic M4/M0
pair has been checked for C2BOOT startup and shared-SRAM communication. The
chip model also provides basic RCC, PWR, HSEM, RNG, and LPTIM register behavior.
Unimplemented peripheral registers read as zero or retain writes.

When `cpu2_firmware` is absent, CPU1 starts the built-in Thread RCP model. Its
IPCC and Spinel behavior is serviced in CPU2's 32 MHz time domain while the M0
core runs a wait loop; this mode lets the Zephyr CPU1 image join the example
network and exchange Zigbee data. When a raw CPU2 image is configured, the
built-in RCP model is disabled and that image executes on M0 instead. The
official ST Thread RCP package was checked, but it is FUS-encrypted and is
rejected by the raw-image loader. The STM32WB wireless PHY and radio/baseband
peripherals are not modeled, so an arbitrary wireless stack running from a raw
CPU2 image cannot perform on-air operations without a matching radio model.

The Zephyr source used for the join run also needed a null-index guard in
`subsys/zigbee/nwk/nwk_addr_map.c`: the end-device leave path calls
`zb_ExtAddrByShortAddr` with a null `idx`, while that function previously wrote
through it unconditionally. With that guard, the Zephyr CPU1 image joined the
TLSR8258 sample gateway at about 7.8 seconds and exchanged bidirectional NWK
traffic in the 24-second capture through the built-in CPU2 Thread RCP model. The example is
[`../examples/stm32wb55_zigbee.yaml`](../examples/stm32wb55_zigbee.yaml); it
needs the Zephyr CPU1 image and TLSR8258 coordinator image. Add
`cpu2_firmware` only when testing an independently executable raw M0 image.
