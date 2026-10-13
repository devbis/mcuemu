# Unicorn and TLSR8258 integration

Unicorn is the CPU engine for this project. TC32 is implemented in the separate
Git checkout at `./engine/unicorn`, where its CPU state, translator, register API,
architecture dispatch, bindings, and unit tests are maintained. The root
project builds that checkout with `UNICORN_ARCH=tc32` and links its `unicorn`
target into `mcuemu`.

The TLSR8258 wrapper creates `UC_ARCH_TC32` with `uc_open`. Flash is mapped as
read/execute memory, SRAM is backed by the board's aligned SRAM storage, and
the register window is connected with `uc_mmio_map`. Register access and
execution go through `uc_reg_*` and `uc_emu_start`. Device timing and IRQ
delivery are board responsibilities around those Unicorn calls.

The TC32 backend exposes `uc_tc32_set_fetch_alias` for chips that execute from
a flash instruction-cache window mapped into the CPU address space. The
TLSR8258 wrapper maps fetches from `0x840000` through the selected flash
capacity to flash offset zero; regular data reads and writes in the same
addresses still access SRAM.

## Source checkout

The engine is intentionally kept as a separate Git checkout under `engine/`.
Bootstrap it with:

```sh
./scripts/bootstrap-unicorn.sh
```

The script keeps the checkout at `./engine/unicorn`. When
`TC32_UNICORN_FORK_URL` is set, that URL is configured as `origin`; the
official Unicorn repository is configured as `upstream`. Without a fork URL,
the official repository is configured as `upstream` and no `origin` is added.

## Current boundary

The Unicorn backend is the only production instruction interpreter. The root
wrapper owns TLSR8258 memory and peripherals and calls the Unicorn C API; it
does not decode or execute TC32 instructions itself. The wrapper currently
provides a `0xcfxx` semihosting stop marker for board tests. That marker is a
wrapper convention and is not claimed as a TC32 ISA instruction.

This is an initial TC32 backend and board model. A two-node run with local
`sampleLight` and an SDK-built `sampleGW` coordinator executed without a CPU
fault. Starting the light at 5 seconds produced 70 Zigbee transmissions and 36
successful receptions. The packet capture contains a coordinator Beacon
Response, the light's association request, the coordinator's association
response, and subsequent bidirectional traffic. SRAM inspection at 11.25
seconds showed `zb_isDeviceJoinedNwk()` true on both nodes: the light's
`g_zbNwkCtx + 0x2d` byte is `0x04`; the coordinator byte is `0x96` and also has
`permit_join` and `joinAccept` set.

The join blocker was an RX DMA layout mismatch. The TLSR8258 DMA length field
includes the two FCS bytes, and the SDK subtracts them before parsing the MAC
frame. The model now reserves and counts those bytes while the virtual medium
continues to carry the MAC frame without generated FCS. The earlier
`treti {pc}` correction also remains required: it consumes the saved IRQ
return address from the IRQ stack before restoring the saved CPU mode. Joining
is now verified for this firmware pair on channel 15; complete vendor ISA and
peripheral coverage remain unestablished. BLE stays deferred.
