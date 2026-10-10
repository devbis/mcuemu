# Unicorn and TLSR8258 integration

Unicorn is the CPU engine for this project. TC32 is implemented in the separate
Git checkout at `./engine/unicorn`, where its CPU state, translator, register API,
architecture dispatch, bindings, and unit tests are maintained. The root
project builds that checkout with `UNICORN_ARCH=tc32` and links its `unicorn`
target into `tc32emu`.

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
`sampleLight` and an SDK-built `sampleGW` coordinator executed 200 million
instructions on each node without a CPU fault. The packet-level medium recorded
47 Zigbee transmissions and 32 successful receptions. The SDK
`zb_isDeviceJoinedNwk()` reads bit 2 at `g_zbNwkCtx + 0x2d`; SRAM dumps showed
the coordinator joined (`0x96`) and the light remains factory new (`0x01`). The
coordinator's state byte also has `permit_join` and `joinAccept` set, but the
capture has no MAC beacon response or association request.
SDK diagnostics show zero RX CRC failures, 16 RX IRQs at the coordinator, 12
at the light, and no coordinator join indication.
Correct `treti {pc}` behavior fixed the earlier false return into a task-queue
epilogue: it consumes the saved IRQ return address from the IRQ stack before
restoring the saved CPU mode. This verifies firmware-driven RF DMA and
bidirectional packet delivery. The coordinator formed a network and accepts
joins, but the light did not join during this run. Complete vendor ISA coverage
remains unestablished. Continue by inspecting Beacon Request handling and the
corresponding radio behavior; keep BLE deferred.
