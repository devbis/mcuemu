# PHY62xx (PHY6222 / PHY6252): chip reference

This page describes the chip as it is known after bringing up Zephyr on real boards, and is the
reference for what the emulator model has to do. [`phy6252-investigation.md`](phy6252-investigation.md)
is the log of how the model was grown from images and ROM traces; where the two disagree, this page
is newer, and section 11 lists what changed.

Evidence levels used below:

| Tag | Meaning |
|-----|---------|
| **HW** | Observed on a real board (Ai-Thinker PB-03F, PHY6222 rev 6222M005, 512 KiB flash, flash ID `1340c4`; the ChinaLCTech ECB02C, 256 KiB, for the items marked so) |
| **ROM** | Read from the 96 KiB boot ROM dump (`phy6222-rom.bin`, ROM addresses `0x0..0x17fff`) |
| **SDK** | Taken from the PHY62XX SDK 3.1.1 sources or its `bb_rom_sym_m0.txt` |
| **inferred** | Deduced, not seen; do not rely on it |

The Zephyr port is in `soc/phyplus/phy62x2`, `dts/arm/phyplus/phy62x2.dtsi` and the drivers named
in each section.

## 1. Overview

- Arm Cortex-M0, 2 priority bits, no VTOR, 32 interrupt lines. Core clock 16 MHz (XTAL) or 48 MHz
  (DLL), chosen through the ROM. The PHY6252 and PHY6222 are the same programming model; boards report
  themselves as PHY6222.
- 64 KiB SRAM in three blocks, execute-in-place flash (SPIF), 96 KiB boot ROM at address zero.
- 2.4 GHz radio with IEEE 802.15.4 support (the baseband is shared with BLE), hardware AES.
- Peripherals: 2 UART, 2 I2C, 2 SPI, GPIO (23 pads in the controller), PWM (6 channels), ADC, QDEC,
  4-channel DMA, timers 1..6, watchdog, always-on (AON) block with RTC and retained registers, key
  scan, RNG.
- The boot ROM contains the whole link layer. The firmware calls it through a jump table in SRAM, and
  the ROM calls the firmware through the same table (section 4).

## 2. Memory map

| Range | What |
|-------|------|
| `0x00000000..0x00017fff` | Boot ROM (96 KiB); also visible at `0x10000000` |
| `0x11000000..` | XIP flash through the cache; 512 KiB on the PB-03F (256 KiB on the ECB02C) |
| `0x22000000..` | The same flash, uncached alias; the ROM loader reads images through it |
| `0x1fff0000..0x1fff7fff` | SRAM block 0, 32 KiB |
| `0x1fff8000..0x1fffbfff` | SRAM block 1, 16 KiB |
| `0x1fffc000..0x1fffffff` | SRAM block 2, 16 KiB |
| `0x40000000..` | Peripherals (table in section 7) |

SRAM layout used by the ROM (**ROM**, **HW**):

- `0x1fff0000..0x1fff03ff`: the jump table, 256 words (section 4). The vectors of the running
  program are taken from it, not from address zero.
- `0x1fff0a08`, `0x1fff0a44`, `0x1fff0a4a`, `0x1fff0a80` and the rest below `0x1fff1838`: ROM
  variables. The ones that matter: `0x1fff0a0c` `sleep_flag` (marker `0x032141b6`),
  `0x1fff0a44/45` RF calibration `g_rfPhyTpCal0/1`, `0x1fff0a4a` the system clock selector the
  firmware writes before `clk_init`, `0x1fff0a80` `g_system_reset_cause`.
- `0x1fff1838`: the run descriptor (stack pointer, reset address) that the ROM copies from the image
  table; the usable SRAM of an application starts at `0x1fff1840` and ends at `0x20000000`.

All three SRAM blocks hold the Zephyr image (code in SRAM for the flash driver, data, stack), so a
sleep that must keep the program retains all three (section 6).

## 3. Boot

### 3.1 Reset path (**ROM**, **HW**)

The reset handler (`0xd90`) first calls `0x850d`, which reads `AON->SLEEP_R[0]` (`0x4000f0c0`),
stores the reset cause in `g_system_reset_cause` and writes `1` back into `SLEEP_R[0]`. Then it
reads `AON 0x4000f0a8` (the register the SDK calls `REG_S11`) and tests **bit 0, the sleep flag**:

- bit 0 clear: cold start path (`0xc1`). Opens the UART loader window unless the cause says
  otherwise, then runs the image.
- bit 0 set: warm path (`0x176a9`). It calls `0x1532c`, then **calls `jump[62]`
  (`WAKEUP_PROCESS`, `0x1fff00f8`) if it is not zero**; with a zero slot it continues with the
  ROM's own wakeup of the BLE controller.

The flag is set and cleared by the ROM function `set_sleep_flag(x)` (`0x16c01`): x non-zero sets
bit 0 of `0x4000f0a8` and stores `0x032141b6` into `sleep_flag`; zero clears both. The warm path
checks the marker in `sleep_flag` and clears the flag when it is missing, so a flag set by itself
(RAM lost) falls back to a cold start.

Reset causes, as `SLEEP_R[0]` is turned into `g_system_reset_cause` (**ROM**; the values seen on
the board in brackets are **HW**):

| `SLEEP_R[0]` before reset | cause | Meaning | Loader window |
|---------------------------|-------|---------|---------------|
| 0 | 0 `RSTC_COLD_UP` | Power-up, or a firmware reset that wrote 0 [observed after a power cycle: 0] | opens |
| bit 0 (1) | 1 `WARM_UP` | Warm start | opens |
| bit 1 (2) | 2 `OFF_MODE` | After the power off / the sleep with RAM kept | skipped |
| bit 2 (4) | 5 `WARM_NDWC` | Warm start of the user mode (the deep sleep of the SDK) [observed after the deep sleep: 5] | skipped |

`SLEEP_R[0]` reads `1` after any ROM start, because the ROM writes it back. After a suspend to RAM
the register still reads `2`: the warm path does not rewrite it [**HW**].

`SLEEP_R[0..3]` (`0x4000f0c0..0x4000f0cc`) and `0x4000f0a8` survive sleep and soft reset. They do
not survive a power loss.

### 3.2 UART loader of the ROM (**HW**)

- Sync is at **9600 baud** on **P10** (the RX pad; edge capture), the tool sends the string
  `UXTDWU` until the ROM answers `cmd>>:`; afterwards the link runs at 115200. Probing at 115200
  never finds the ROM.
- The window is very short after a reset: the first `UXTDWU` has to go out about 50 ms after the
  reset (0.3 s is too late). It is skipped completely for the causes 2 and 5.
- `rdwr_phy62x2.py -p PORT we 0x11002000 image` writes the first image; flash offset `0x2000`
  is where the ROM looks for the segment table. Writing at offset 0 "works" but never boots.
- A running application gets back into the window with the SDK reset: write `0` to
  `SLEEP_R[0]` and `0` to `PCR->SW_RESET1` (`0x40000004`), which resets every module. A bare
  `NVIC_SystemReset()` resets only the core and the ROM does not run afterwards.
- Never clear the P9/P10 pad multiplexer (`0x4000380c`) before a reset: the board hung and needed
  a power cycle.

### 3.3 Image format (**SDK**, **HW**)

The ROM looks for a table at flash offset `0x2000`:

| Offset in the table | Content |
|---------------------|---------|
| `+0x00` | Number of entries (segments plus the run descriptor) |
| `+0x08` | Address of the run descriptor in SRAM, `0x1fff1838` |
| `+0x100 + 16*i` | Entry: flash offset (from `0x11000000`), size, destination, CRC field |

A destination in `0x11xxxxxx` is an XIP segment (the code stays in flash); a destination in SRAM is
copied by the ROM. The last entry is the run descriptor: the initial stack pointer and the Thumb
reset address, copied to `0x1fff1838`. The packer writes `0xffffffff` into every CRC field and the
ROM accepts it. The ROM adds `0x11000000` to the stored offset and reads through `0x22000000`.

Hardware constraint: **an image whose XIP code starts below `0x11010000` does not start**. The code
of the loader and of a plain application is at `0x11010100`, behind the table.

`soc/phyplus/phy62x2/make_image.py` builds this image from an ELF; `merge_loader.py` joins a loader
and a signed application into one image for the first flashing.

### 3.4 Flash layout of the PB-03F (**HW**)

```
0x002000  ROM segment table
0x010000  MCUboot, XIP at 0x11010100, 0x7f00 bytes
0x018000  slot 0 (0x32000): MCUboot header 0x200, then the application
0x04a000  slot 1 (0x32000)
0x07c000  storage (NV of the application), 0x4000
```

The application is linked at `0x11018200` and started by MCUboot, which is itself started by the
ROM. Swap using move works: a 65 KB image uploaded to slot 1 through the serial recovery of
MCUboot at 115200 baud (about 28 s) was swapped in and started, repeatedly.

## 4. The jump table (**SDK**, **ROM**, **HW**)

`JUMP_FUNCTION(n)` is word `n` of `0x1fff0000`. ROM code calls the entry when it is not zero and its
own code otherwise. The slots the port touches:

| Slot | Address | Use |
|------|---------|-----|
| 62 | `0x1fff00f8` | `WAKEUP_PROCESS`: run by the warm path after a sleep with the flag set |
| 63 | `0x1fff00fc` | override of `config_RTC` |
| 64 | `0x1fff0100` | override of `enter_sleep_off_mode` |
| 219..223 | `0x1fff036c..` | NMI, HardFault, SVCall, PendSV, SysTick handlers |
| 224..255 | `0x1fff0380..` | interrupt handlers of the lines 0..31 |
| 228 | `0x1fff0390` | baseband (line 4); the ROM handler calls it when not zero |
| 240 | `0x1fff03c0` | GPIO (line 16) |
| 244..249 | | timers 1..6 (lines 20..25) |

The ROM vector table at zero points into the ROM; each ROM vector reads its slot at
`0x1fff0380 + 4*n` and calls it. The Zephyr port fills slots 219..255 from its own vector table in
`soc_prep_hook()`, so Zephyr's handlers run without any VTOR.

## 5. Clock, reset and the flash cache

### 5.1 Clock gates and resets (**SDK**, **HW**)

`PCR` is at `0x40000000`: `SW_RESET0 +0x00`, `SW_RESET1 +0x04`, `SW_CLK +0x08`, `SW_RESET2 +0x0c`,
`SW_CLK1 +0x14`, cache gate `+0x20`, cache reset `+0x24`, cache bypass `+0x28`. A reset bit of the
same module is at the same position as its clock bit in `SW_CLK` (a module is reset by clearing and
setting its bit in `SW_RESET0`).

| `SW_CLK` bit | Module | `SW_CLK` bit | Module |
|--------------|--------|--------------|--------|
| 3 | DMA | 15 | QDEC |
| 7 | IOMUX | 17 | ADC controller |
| 8 | UART0 | 18 | PWM |
| 9 | I2C0 | 21, 22 | timers 5, 6 (share reset bit 5) |
| 10 | I2C1 | 25 | UART1 |
| 11 | SPI0 | `SW_CLK1` bit 5 | watchdog |
| 12 | SPI1 | `SW_RESET2` bit 10 | baseband / link layer |
| 13 | GPIO | | |

The SPI controllers act as masters when the matching bits are set in `0x4000302c`
(`AP_COM->PERI_MASTER_SELECT`: bit `n` and bit `n+4` for SPI `n`).

### 5.2 Core clock (**ROM**, **HW**)

- The ROM function `clk_init(sel)` (`0xa5ed`) selects the clock; `sel` is 2 for 16 MHz XTAL and 3
  for the 48 MHz DLL. The firmware first writes the selector to `0x1fff0a4a`. `clk_get_pclk()`
  (`0xa5d1`) returns the resulting peripheral clock.
- The selection is stored in AON `CLKSEL` (`0x4000f03c`) bits 2:0; the ROM table
  (`0x17be0`) lists `32, 32, 16, 48, 64, 96, 8, 4` MHz. Changing the field changes the core clock.
- `CLKSEL` bit 16 selects the 32 kHz source of the watchdog and the RTC: clear is the RC oscillator.
- **The UART divider follows the core clock**: at 16 MHz the divisor for 115200 is 9 and gives
  111111 baud (-3.5 %), which garbles long lines (the serial recovery of MCUboot loses its
  frames); at 48 MHz it is 26 (`DLL 0x1a`, +0.16 %). The boards that carry a console run at 48 MHz.
  The SPI and I2C clocks are the core clock too.

### 5.3 The flash cache is off after the ROM (**HW**, ECB02C)

The ROM leaves the XIP cache bypassed: `PCR 0x40000028 = 1`. Every instruction is then fetched by
a flash transaction and the code runs about a hundred times slower (the hash of a 50 KB slot took
43 s, and looked like a "10 s wait after reset"). The firmware configures the flash read mode and
enables the cache the way the SDK does, from SRAM because the read mode changes underneath:

1. `spif_config(sys_clk, 1, XFRD_FCMD_READ, 0, 0)` (ROM `0x16dc5`);
2. gate `0x40000020 |= 3`, reset `0x40000024 = 2`, short delay, `= 3`;
3. bypass `0x40000028 = 1`; cache control `0x4000c000 = 2`, delay, `= 3`, delay, `= 0` (flush the
   tags); bypass `= 0`.

The same sequence has to be repeated after a sleep that resets the clock tree (section 6.4).

## 6. Power: AON block, RTC, sleep and power off

### 6.1 AON register map (**SDK**, **ROM**, **HW**)

`AON` is at `0x4000f000` and keeps its state through every sleep.

| Offset | Name | Notes |
|--------|------|-------|
| `0x00` | `PWROFF` | write `0x5a5aa5a5` after `SLEEP_R[0]=2`: power off, wake by pads only |
| `0x04` | `PWRSLP` | `0xa5a55a5a` is the key of the system sleep (written by `enter_sleep_off_mode`) |
| `0x08..0x10` | `IOCTL0..2` | pad pulls (two bits per pad) and the wake edge; see 7.1 |
| `0x14` | `PMCTL0` | pulls of P31..P34 and the power/clock control of the sleep (bits 31:27) |
| `0x18` | `PMCTL1` | bias of the 32 kHz crystal (17..18), RC32K cap bank; reads `0x01064040` on a cold PB-03F |
| `0x1c` | `PMCTL2_0` | bit 6 software control of the 32 kHz clock, bits 16:7 clock enables, **bits 21:17 SRAM retention** |
| `0x20` | `PMCTL2_1` | ADC attenuators |
| `0x24` | `RTCCTL` | bit 0 run, bit 1 clear, bits 13:2 prescaler, bits 15/18/20 compare-0 interrupt/overflow/event |
| `0x28` | `RTCCNT` | 24-bit counter, read until two reads agree |
| `0x2c` | `RTCCC0` | compare 0 |
| `0x3c` | `CLKSEL` | system clock selector, bit 16 the 32 kHz source |
| `0x40..0x48` | `CLKHF_CTL0/1`, `ANA_CTL` | high-frequency clock and analog enables (ADC uses them) |
| `0x5c`, `0x64` | `cal_rw`, `cal_ro1` | RC32K calibration |
| `0x6c..` | `ADC_CTL` | ADC analog control |
| `0xa0`, `0xa4` | `REG_S9`, `REG_S10` | wake enables of the pads, one bit per pad number |
| `0xa8` | `REG_S11` | bit 0 is the sleep flag of the ROM |
| `0xc0..0xcc` | `SLEEP_R[0..3]` | retained words; `[0]` is the reset cause for the ROM |
| `0x160..0x17c`, `0x180` | eFuse blocks, `SECURTY_STATE` | |

### 6.2 RTC (**HW**)

- A 24-bit counter on the **RC oscillator, about 32 kHz and not calibrated**. It runs after reset
  (`RTCCTL = 1`). A 65536-tick comparator ended a sleep after the 2 s that were asked for, with
  about 260 ticks of overhead.
- The comparator is armed by the ROM `config_RTC(ticks)` (`0xa6f9`; jump slot 63 overrides it):
  it saves the counter in its variable at `0x1fff0a08 + 0x2c`, calls `0x8900(1)` (a one-tick wait),
  writes `RTCCC0 = counter + ticks` and sets `RTCCTL |= 0x148000` (bits 15, 18, 20).
- When the counter passes `RTCCC0` the compare event **sets the interrupt line 6 pending in the
  NVIC** [observed: `NVIC_ISPR` bit 6]. `RTCFLAG` (`0x38`) stays 0 and is not the event flag.
  Nothing in the SDK or the ROM handles the interrupt; the compare exists to wake the chip.

### 6.3 The RC oscillator must be set up for the sleep (**HW**)

The RTC counts and its comparator matches, but **the chip does not wake from any sleep unless the
32 kHz clock is configured the way the SDK's `hal_rtc_clock_config(CLK_32K_RCOSC)` does at start**.
A cold-booted chip has `PMCTL0 = 0x00268000`, `PMCTL1 = 0x01064040`, `PMCTL2_0 = 0x0001ffab` and
never woke from the deep sleep, the suspend or a bare ROM sleep sequence. The SDK sequence, which the port runs in
`soc_prep_hook()` when a sleep is enabled:

```
PMCTL0[31:27]   = 0x05
PMCTL2_0[16:7]  = 0x3fb          (bit 9 of the register goes to 0)
PMCTL2_0[6]     = 1              (software control of the 32 kHz clock)
PMCTL0[28]      = 1              (the comment in the SDK: "for rc32k wakeup")
PMCTL1[18:17]   = 0
```

`PMCTL0` then reads `0x38268000`. The sleep entry clears `PMCTL2_0` bit 6 again. **Do not clear
`PMCTL0[29:27]` before the sleep**, as the SDK's `hal_pwrmgr_enter_standby()` appears to: the chip
then never wakes. The SDK's wakeup function sets the field to 7 again.

### 6.4 Sleep modes (**HW**)

All the modes enter through the ROM `enter_sleep_off_mode(mode)` (`0xafa1`; jump slot 64 overrides
it). Mode 1 writes `0xa5a55a5a` to `AON+0x04`, mode 2 writes `~0xa5a55a5a = 0x5a5aa5a5` to `AON+0x00`
(the power off); then both write `0x12345678` to `AON+0x34` and loop. The core stops and the
watchdog stops with it (a chip asleep for 14 s is not reset by an 8 s watchdog).

**Deep sleep (the SDK's `hal_pwrmgr_enter_sleep_rtc_reset`)** — verified:

```
PMCTL2_0[6]     = 0
config_RTC(ticks)
PMCTL2_0[21:17] = retained SRAM blocks (0 for none)
SLEEP_R[0]      = 4                 -> the ROM boots with the cause 5, no loader window
enter_sleep_off_mode(1)
```

The chip wakes on the RTC, starts from the ROM as after a reset (no flag, cold path) and runs the
image again. The peripherals and every SRAM block that is not retained are lost; the retained
blocks keep their contents [**HW**: a `zephyr,retained-ram` counter of 1 survived]. With
`SLEEP_R[0] = 0` and no retention the wake is an ordinary cold start including the loader window
[**HW**].

**Suspend to RAM (the SDK's standby, resumes the program)** — verified:

```
save: clock gates, NVIC, SHPR2/3, SysTick, pin multiplexer (IOMUX 0x40003800..0x4000382c),
      the registers of the program (setjmp-like)
jump[62]        = wakeup function (Thumb address, in SRAM)
config_RTC(ticks)
set_sleep_flag(1)
PMCTL2_0[6]     = 0
PMCTL2_0[21:17] = 7                 (all three SRAM blocks)
SLEEP_R[0]      = 2
enter_sleep_off_mode(1)
```

On the wake the ROM's warm path (flag set) calls `jump[62]` on its own stack, with the core clock
back at 16 MHz, all clock gates at their reset state, the peripherals and the pin multiplexer
reset, the NVIC empty and the SysTick stopped. The wakeup function:

1. restores `SW_CLK1` and `SW_CLK` first (**the RTC and the AON registers are read through those
   gates**; reading the RTC before them hangs);
2. `PMCTL0 |= 7 << 27`, waits 160 RTC ticks (5 ms, as the SDK does);
3. `set_sleep_flag(0)`, clears `jump[62]`;
4. `clk_init` for the board clock, restores `SW_CLK`/`SW_CLK1` again, restores the pin
   multiplexer, enables the flash cache (5.3);
5. restores the NVIC priorities and enables, the SHPR registers and the SysTick, then returns into the
   saved program context with the interrupt state it had.

A 2 s and a 0.5 s suspend both returned, with the stack, the program counter and the shell where
they were. The UART lost its divisor, FIFO and interrupt enable and the driver had to configure it
again, **including `IER`**: configuring a UART disables its interrupts, so after a resume the shell
transmitted but no longer received until the driver wrote `IER` back. Zephyr's kernel clock
(SysTick) does not run in the sleep, so `k_uptime_get()` shows no gain unless the power management
adds the RTC time.

**Power off** (`sys_poweroff`) writes `SLEEP_R[0] = 2` and the key to `AON+0x00`; only the wake pads
start the chip. Written from the SDK; **not run on the board**.

### 6.5 Wake pads (**SDK**, not run)

A pad that wakes the chip has: its pull set (strong pull-up for a falling edge, pull-down for a
rising one), the edge bit in its pull register, the edge-sensitive mode in the GPIO controller
(`inttype_level`, `0x40008038`) and its bit in `REG_S9`/`REG_S10` (`0x4000f0a0/a4`), numbered by
**pad number**, not by controller index (7.1). The port arms the pads of the
`phyplus,phy62x2-poweroff-wakeup` devicetree nodes and the pads that have a GPIO interrupt enabled.
The wake by a pad was tried once, with the RC oscillator not set up, and did not wake; **it has not
been tried since**.

## 7. Peripherals

### 7.1 Pads, IOMUX and GPIO (**SDK**, **HW** for the UART pads)

The GPIO controller has 23 pins numbered 0..22 (the "pin index" below); the pads have the numbers
P0..P34 of the data sheet. The wake enables, the pulls and the analog switches use pad numbers or
indexes as follows:

| Index | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 | 22 |
|-------|---|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|----|----|----|----|----|----|
| Pad   | P0 | P1 | P2 | P3 | P7 | P9 | P10 | P11 | P14 | P15 | P16 | P17 | P18 | P20 | P23 | P24 | P25 | P26 | P27 | P31 | P32 | P33 | P34 |

P9 and P10 are the ROM UART (TX, RX); P2 and P3 are the pads of the debug interface; P16/P17 are the
pads of the 32 kHz crystal.

Registers (`IOMUX` at `0x40003800`, clock gate `SW_CLK` bit 7):

- `+0x00` analog enable: bit `n` for the pads P11.. (indexes 7..16): clear it for digital use;
- `+0x0c` full-mux enable: bit per index; set it to give the pad to a peripheral function, clear it
  for GPIO;
- `+0x14` pad enable: bit `n` of P2 and P3 gives the pad to the peripherals;
- `+0x18..` `gpio_sel`: one byte per index, `0x40003818 + 4*(index/4)`, the function number in the
  low six bits;
- `+0x44` PS0 (ADC pads).

Pulls: two bits per pad in `AON` `IOCTL0..2` and `PMCTL0`: 0 floating, 1 strong pull-up, 2 pull-up,
3 pull-down (the weak pull-up is what a UART pad uses). The register and the shift of each index are
in `pinctrl_phy62x2.c` and `wakeup.c`.

GPIO (`0x40008000`, IRQ 16, `SW_CLK` bit 13): DesignWare APB GPIO, `DATA +0x00`, `DIR +0x04`,
`INTEN +0x30`, `INTMASK +0x34`, `INTTYPE +0x38` (1 edge), `INTPOLARITY +0x3c`, `INTSTATUS +0x40`,
`EOI +0x4c`, `EXT_PORT +0x50`. One polarity per pin, so an interrupt on both edges re-arms to the
opposite level after every edge; the driver re-reads the level after the write.

**UART pads on the boards** (**HW**): the PB-03F console is UART0 on indexes 5 and 6 (P9 TX, P10 RX)
with weak pull-ups, which are also the pads of the ROM loader. The ECB02C console is UART1 on
indexes 3 (TX) and 2 (RX) with strong pull-ups, and keeps P9/P10 as the service pads for the ROM
(the ROM sync works only there).

### 7.2 UART (`0x40004000` UART0, IRQ 11; `0x40009000` UART1, IRQ 17)

NS16550-compatible, 32-bit access, register spacing 4. After the NS16550 driver configures it the
FIFO bit of `FCR` is clear and it raises one interrupt per character, which loses bytes in a burst at
115200 with the latency of this core; the port writes `FCR = 7` again. The module loses everything in
a sleep (6.4).

### 7.3 I2C, SPI (DesignWare; built, **not run**)

- I2C0 `0x40005000` IRQ 12, I2C1 `0x40005800` IRQ 13; DW I2C, input clock = core clock
  (`I2C_DW_CLOCK_SPEED` 16 or 48).
- SPI0 `0x40006000` IRQ 14, SPI1 `0x40007000` IRQ 15; DW SPI. The FIFO depth is not in the SDK; 8 is
  assumed (the SDK's threshold of 4 needs at least that). The master select is `0x4000302c`.

### 7.4 PWM (`0x4000e000`; built, **not run**)

Six channels. `+0x00` enable (bits 0 and 4), per channel `n`: `CTRL0 = +0x04 + 12n` (bit 0 enable,
bit 4 polarity, the divider from bit 12, bit 16 load, bit 31 instant load), `CTRL1 = CTRL0 + 4`
(compare/total counts). The clock is assumed to be 16 MHz. Pads come from pinctrl.

### 7.5 ADC (`0x40050000`, IRQ 29; built, **not run**)

Polling, high-resolution mode only. Single-ended channels 2..7, differential pairs 1, 3, 5, 7. The
analog side is in AON: `ANA_CTL` (`+0x48`) bits 0 (LDO), 3 (ADC), 23 (mic bias), `ADC_CTL`
(`+0x6c + 4n`), `CLKHF_CTL0/1`, `PMCTL2_1` for the attenuator, and the pad analog switch
(`IOMUX +0x00`). Samples of block `n` are at `ADC_BASE + 0x400 + 0x80n` (the first two words are not
data), 29 samples per conversion, reference 0.8 V. Calibration and the 3.2 V range are not done.

### 7.6 QDEC (`0x4000b000`, IRQ 30; built, **not run**)

Three channels X, Y, Z: `+0x00` enable, `+0x04` interrupt enable, `+0x08` interrupt clear, per
channel `n` mode `+0x10 + 0x14n`, count `+0x1c + 0x14n`. Clock gate bit 15.

### 7.7 DMA (`0x40010000`, IRQ 19; built, **not run**)

DesignWare AHB DMA, four channels, `0x58` bytes of registers each (`SAR +0x00`, `DAR +0x08`,
`CTL_L +0x18`, `CTL_H +0x1c`, `CFG_L +0x40`, `CFG_H +0x44`), common registers at `+0x2c0`
(`RAW_TFR`) ... `+0x398` (`DMA_CFG`), `+0x3a0` (`CH_EN`). The handshake interfaces are in the classic
`CFG_H` fields (bits 10:7 source, 14:11 destination), unlike the Intel layout of the Zephyr DW
driver. Channel 0 moves up to `0x7ff` items per block, the others `0x1f`. The ROM helper at `0xa92c`
is a channel-0 memory copy (`CTL = 1`, `CFG_H = 4`).

### 7.8 Timers and the watchdog

- Timers 1..6 share the DW timer layout, 4 MHz source, IRQs 20..25. Timer 5 at `0x40001050` and
  timer 6 at `0x40001064` are used by Zephyr (`SW_CLK` 21 and 22). User-defined mode reloads at zero
  and keeps running until stopped.
- Watchdog (`0x40002000`, DW APB WDT, 32 kHz RC, IRQ 10; **HW**): the clock gate is `SW_CLK1` bit 5,
  the resets `SW_RESET0` bit 2, `SW_RESET2` bits 2 and 5, the clock source `CLKSEL` bit 16 clear.
  **`reset-pulse-length` must be 256** (the SDK writes `CR = 0x1d`); with 0 the timeout expires and
  the chip does not reset. An 8 s timeout with the feeder at the lowest priority resets a hung chip
  after about 7 s (`irq_lock()` and a loop). Initialisation must not call `k_busy_wait` before the
  SysTick runs.

### 7.9 Flash controller (SPIF `0x4000c800`, IRQ 18) (**HW**)

The XIP window is read-only. Erase and write go through ROM functions, called from SRAM with the
interrupts masked: `spif_erase(addr)` sector erase (`0x16fa9`), `spif_write(addr, buf, len)`
(`0x17395`), `spif_config` (`0x16dc5`). Erase of a 4 KiB sector took about 20 ms. A flash that
sleeps needs `spif_set_deep_sleep`/`spif_release_deep_sleep` (the SDK under
`CFG_FLASH_ENABLE_DEEP_SLEEP`); the port does not use them. SPIF write protection registers at
`0x4000c850..0x4000c858`.

### 7.10 Radio and AES

- RF block `0x40030000`, link layer `0x40031000`, TX FIFO port `0x40031400`, RX FIFO port
  `0x40031c00`, IRQ 4 (baseband). The ROM `ll_hw_*` functions drive it; the port's 802.15.4 driver
  (`ieee802154_phy62x2.c`) uses RF/LL registers directly and the ROM FIFO reader
  `ll_hw_read_rfifo_zb`.
- LL mode register `+0x04`: `0x000100` STX, `0x010001` SRX, `0x010102` TRX, `0x010103` RTX; IRQ status
  `+0x38` (SDK layout `MD` bit 0, `RTO` bit 2, `COK` bit 10; the compact layout used by an early
  driver was wrong); FIFO pointers `+0x50`/`+0x54`, FIFO reset `+0x58` (bit 0 TX, bit 8 RX).
  TRX settle time at `+0xbc` is `bb << 16 | afe << 8 | pll` (Zigbee 57/8/52, ACK 118/8/52 us).
- CRC format register `0x40030124` must be set to the Zigbee format (3).
- The baseband and link layer reset is `SW_RESET2` bit 10; the analog clock and enable are
  `0x4000f040/44`.
- AES: `0x40040000`, key `+0x20..+0x2c`, data `+0x30..+0x3c` (first four bytes in the highest
  register), store 1 to `+0x00` to encrypt, result `+0x50..+0x5c`; ROM `LL_ENC_AES128_Encrypt0`
  takes `(key, plaintext, ciphertext)`. The port does not use the hardware block.

The Zigbee radio path was exercised on the ECB02C in earlier work; on the PB-03F it was not
re-checked in this bring-up.

### 7.11 Interrupt numbers (**SDK**)

| Line | Source | Line | Source |
|------|--------|------|--------|
| 4 | baseband | 17 | UART1 |
| 5 | key scan | 18 | SPIF |
| 6 | RTC | 19 | DMA |
| 10 | watchdog | 20..25 | timers 1..6 |
| 11 | UART0 | 28 | AES |
| 12, 13 | I2C0, I2C1 | 29 | ADC |
| 14, 15 | SPI0, SPI1 | 30 | QDEC |
| 16 | GPIO | 31 | RNG |

## 8. ROM entry points used by the port

| Address | Function | Verified by |
|---------|----------|-------------|
| `0x00000d91` | reset handler | ROM |
| `0x0000850d` | reads `SLEEP_R[0]`, sets the cause | ROM, HW |
| `0x0000a5ed` / `0x0000a5d1` | `clk_init` / `clk_get_pclk` | HW |
| `0x0000a6f9` | `config_RTC` | ROM, HW |
| `0x0000afa1` | `enter_sleep_off_mode` | ROM, HW |
| `0x00016c01` / `0x0000b0c1` | `set_sleep_flag` / `get_sleep_flag` | ROM, HW |
| `0x00016dc5` | `spif_config` | HW |
| `0x00016fa9` | `spif_erase` (sector) | HW |
| `0x00017395` | `spif_write` | HW |
| `0x00016ab5` | `rtc_clear` | SDK |
| `0x000110b1` | `ll_hw_rst_rfifo` | SDK |
| `0x0000a92c` | DMA channel-0 copy helper | ROM |
| `0x00008900` | RTC tick wait | ROM |

`bb_rom_sym_m0.txt` in the SDK (`misc/`) names the rest.

## 9. What was run on a board, and what was not

Run on the PB-03F (or the ECB02C where marked), with the Zephyr port:

- the ROM loader (9600 baud sync, flash write at `0x11002000`), first image with MCUboot and a shell;
- MCUboot swap-using-move through the serial recovery at 115200 baud, many times;
- 48 MHz core clock, UART at 115200 without errors; the flash cache sequence (ECB02C);
- the shell on UART0 and a console on UART1 (ECB02C); `kernel reboot cold` into the ROM window;
- the watchdog resetting a hung chip; the reset cause variable (0 after a power-up, 5 after a deep
  sleep); `SLEEP_R[0]` readback;
- the RTC counting, its compare setting the interrupt line 6 pending;
- deep sleep with the retained RAM; suspend to RAM of 2 s and 0.5 s with the program resuming.

Written from the SDK or the ROM and **not run**: I2C, SPI, PWM, ADC, QDEC, DMA, the wake by a pad,
the power off, the flash in its deep sleep, the retention of the pad pulls and of the outputs across
a sleep, the Zephyr power-management policy on top of the suspend (`CONFIG_PM`), the radio after a
sleep, the current in any sleep mode, and hardware AES.

Practical notes from the bench:

- The RESET button of the test setup cuts the power of the board. A chip that has been in a sleep
  keeps its state on the residual charge, so a short unplug does not recover it: leave it
  unpowered for 10 to 15 s.
- A hung sleep is silent: the watchdog and the UART both stop, and nothing but a power cycle
  recovers the chip. Do the experiments with a shell that can set the registers one at a time
  (`devmem`, a ROM call by address) and enter the sleep as the last command.

## 10. The emulator model

The model reproduces the sequence of section 6 (the log has the details, "Cold start and system
sleep through the ROM"); `boot_from_rom` runs the dumped ROM's own reset and wake code. What a model
of this chip has to do for the firmware to resume:

1. The RTC wake works only when the RC oscillator is configured (6.3): without
   `PMCTL0[28:27]`, `PMCTL2_0[16:7] = 0x3fb` and `PMCTL1[18:17] = 0` the chip must not wake.
2. `PWRSLP = 0xa5a55a5a` (and the `0x12345678` write to `AON+0x34`) starts the sleep; `PWROFF =
   0x5a5aa5a5` the power off.
3. On the wake, `REG_S11` bit 0 decides the path: set, call `jump[62]` on the ROM stack with
   the core clock at 16 MHz and the peripherals, the pin multiplexer and the NVIC in their reset
   state, the SRAM kept per `PMCTL2_0[21:17]`; clear, the cold path.
4. A deep sleep (`SLEEP_R[0] = 4`) wakes into the ROM start with the cause 5; the ROM writes
   `SLEEP_R[0] = 1` and skips the loader window.
5. The RTC compare sets the NVIC line 6 pending; `RTCFLAG` stays 0.
6. The UART divisor and `IER` are lost in a sleep, the clock gates return to their reset values.

The ROM's `set_sleep_flag` checks the marker `0x032141b6`; a model that keeps the flag in `REG_S11`
alone is enough unless RAM is lost.

## 11. Differences from the investigation log

- The log's RTC wake needed "both compare-0 interrupt and event enables". The hardware needs, in
  addition, the RC oscillator configuration of 6.3, which the log never mentions.
- The log says `PWRSLP` waits for the compare and resets "through the external ROM vector". The ROM
  runs `jump[62]` only when `REG_S11` bit 0 is set; otherwise it is a cold start (and with cause 5 or
  2 it skips the loader window).
- `config_RTC` is not just a return-only shim: it also waits one RTC tick (`0x8900(1)`) and keeps the
  counter in a ROM variable; `enter_sleep_off_mode` also writes `0x12345678` to `AON+0x34`.
- `SLEEP_R[0]` reads 1 after every ROM start; after a suspend it keeps 2.
- The wake enables of `REG_S9`/`REG_S10` use pad numbers (table in 7.1), not indexes of the GPIO
  controller.
- The log took the flash ID `0x1364c8` from a TS0001 module; the PB-03F has `1340c4`, 512 KiB.
- The UART loader of the ROM syncs at 9600 baud, its window is ~50 ms after the reset, and it is
  skipped for the causes 2 and 5; a model that always opens it differs from the board.
- The core clock is 16 MHz after the ROM, so the UART divisor of a 48 MHz board is wrong at the start
  of every wake until `clk_init` runs; the first bytes after a resume come out at the wrong rate.
