# STM32WB55 CPU2 802.15.4 radio: register interface map

Derived only from ST's decompiled Thread RCP v1.13.0 (`thread/src/*.c`, literal
pool, strings) and cross-checked against the PHY v1.25.0 decompile (which has a
CLI whose log strings name several fields). No ST source or datasheet table was
used. Facts here are register semantics, not code.

Tags: **OBS** = seen in code (function address, literal). **INF** = inferred,
with the reason. Function addresses are the Ghidra `FUN_xxxxxxxx` names, hex,
no prefix. `Rxxxx` below means the 32-bit-slot register at `0x60001000 + 0xxxx`
(only the low 16 bits are used by the firmware).

Decompiler pitfall that cost time: `DAT_x[N]` indexes scale by 4, but
`*(T*)(DAT_x + off)` offsets are bytes. All addresses below were recomputed.

## 1. Address map (CPU2 APB3 window)

| Range | Block | Evidence |
| --- | --- | --- |
| `0x6000000c` | RF-register bus ("RadSPI") command word | OBS 89bc, 89ee, 8af4, 8b80, 8c0c |
| `0x60000400..0x6000041b` | radio controller (power, 32 kHz counter, IRQ) | OBS 7cd4, 7d4c, 81fc; PHY `dump_radio_regs` names them |
| `0x60001000..0x600013ef` | 16-bit MAC registers, `+4*index` (index 0..0xfb) | PHY dump loop `< 0xfc`; "Reg offset (= 4* reg addr) -> 16 bits data" |
| `0x600013e8..0x600013fc` | RX FIFO port, FIFO status, command port | OBS 52e6, 519c, 5128 |
| `0x60001400..0x600017ff` | write-1-to-**set** alias of `0x60001000..0x600013ff` | INF (below) |
| `0x60001800..0x60001bff` | write-1-to-**clear** alias of the same | INF (below) |

Alias evidence: `0x60001100` bit0 is read as "RX on" (4250, 6af0); RX is turned
on by writing 1 to `0x60001500` and off by writing 1 to `0x60001900`
(4250: both, then polls RF state). The PHY firmware uses the base pair
`0x60001400+0xec` / `0x60001800+0xec` (fns 8ad8, 8afc) as set/clear of
`R0ec`, and `0x600015cc`/`0x600015d4` against `0x600011cc`/`0x600011d4`. Use
it as: `SET: R |= v`, `CLR: R &= ~v`. Never seen applied above offset `0x1ff`.

Not used by the lld: `0x60000000..0x60000400` except `0x0c`, and
`0x60000800..0x60000fff`.

### 1.1 Radio controller `0x60000400` (names from PHY dump strings)

| Addr | PHY name | Use |
| --- | --- | --- |
| `0x60000400` | Rcc Reg | PHY CLI only: `|=0x8000` / clear, `|=1` / clear (INF: debug/DTB/enable bits) |
| `0x60000404` | Control Reg | power request. 7d4c/7cd4: write `3` (RF on) or `1`; power down writes `2`/`0` (OBS). INF: bit0 = RF domain power, bit1 = second option (written 2 when `param_1==1` selects flavour) |
| `0x60000408` | Dummy | unused |
| `0x6000040c` | Clk32Cnt | slow-clock calibration counter (81fc reads `[0]`, uses `*(+4)`); not needed |
| `0x60000410` | Clk32Period | same |
| `0x60000414` | Clk32Freq | same |
| `0x60000418` | Interrupt Reg | see 3.4 |

### 1.2 `0x6000000c`: RF-register bus (OBS)

Two uses of the same word:

1. **Probe/handshake** (7eea, 7eb4, 819a, 7fc0, 803c): write `0xa5a5`, read it
   back. With `0x60000404` bit0 set it must read back `0xa5a5` (retried up to
   1000 times, 10 us apart; else error 0x62). After a power-down write
   (`0x60000404 = 2`) the same write must read **0** (7eb4 polls for 0, error
   0x61). INF: the word is only writable while the RF domain is powered; this is
   how firmware detects it. Model: `0x6000000c` retains writes with bit16 clear
   only if `0x60000404` bit0 is 1, else reads 0.
2. **Descriptor command** (89ee/8af4/8b80/8c0c): write
   `(descriptor_addr & 0xffff) | 0x10000`; hardware clears **bit16** when done
   (firmware polls bit16, timeout 10 ms, error 0x65). `descriptor_addr` is an
   offset from `0x20030000` (all descriptors are on the CPU2 stack, SRAM2). The
   descriptor in shared RAM:

   | Byte | Meaning |
   | --- | --- |
   | 0 | `n+1`, or `0x80 | (n+1)` for a **read** (`n` data bytes) |
   | 1 | first RF register index (auto-increment) |
   | 2..n+1 | data (read: pre-filled with 0xa5, overwritten in place) |
   | n+2 | 0 terminator |

   Index range checked by firmware: `< 0x7d`, `idx + n <= 0x7d` (fn 89ee). So
   the RF register file is 0x7d bytes, indices 0..0x7c. Single-byte write =
   `{02, idx, val, 00}`, single-byte read = `{82, idx, a5, 00}`.

### 1.3 RF register file (indirect, 8-bit; OBS use, INF names)

| Idx | Use (OBS) | Notes |
| --- | --- | --- |
| `0x04` | channel: `(ch-11)*2` (7fc0; ch 11..26, early-out if unchanged) | range check `ch-11 < 16` else error 2 |
| `0x05` | TX power: bits7:3 = table code (803c); bring-up also `|= 6` | see 2.5 |
| `0x06` | bring-up `|= 1` | |
| `0x0e` | bring-up RMW `= 2 | (old & ~0x1e)`; read after init (3794): bit0 set -> bits2:1 (`0`:8, `2`:0x10, `3`:0x14, else 0xc) | clock/xtal flavour; feeds `R18c`. Read as 0 is acceptable |
| `0x21`,`0x22`,`0x3c` | only for mode `0x2000` (not 0x2001/0x2003): `0x21=0x81`, `0x22 = 8|(old&~0xf)`, `0x3c = 0x80|(old&~0x1f)` | skip for Thread mode 0x2001 |
| `0x2d` | bring-up `= 0x60` | |
| `0x39` | bring-up `= 0` | |
| `0x41` | bring-up `= 0x80 | (old & ~0x40)` | |
| `0x47`,`0x53`,`0x54` | OTP trim from `0x1fff7640..` (82bc): written only if byte pairs are bitwise complements | skip if OTP erased |
| `0x5c` | **state**, `&0x1f`: `0x04` idle/ready, `0x1a` RX on (OBS 4250, 80d4, 8134) | firmware polls it after every state change |
| `0x5d` | PHY event cause (IRQ30): bit0 -> `RxRfCalibrationRetry` cb (6c92), `&0x22`, `&0x14`, bit3 -> other cbs (81fc) | INF: PLL-lock/calibration events; keep 0 |
| `0x65..0x79` | 21-byte table `4c 85 06 1a 72 0f 00 00 00 90 00 01 02 03 04 05 0d 1d 2d 3d 80` (7d4c via 8b6c; table at flash `0x7e9c`) | OBS values |

Bring-up RF writes are in 7d4c; RF reg meanings other than state/channel/power
are INF and irrelevant to a model that returns written values.

## 2. MAC registers and sequences (direct window)

### 2.1 Register table (`Rxxxx` = `0x60001000+xxxx`)

| Addr | Meaning | Tag |
| --- | --- | --- |
| `R000` | sequencer status. `&0x1f==0`: idle (polled by 4200 before state changes, timeout 12 ms). `&7` nonzero blocks FIFO write / TX trigger (45fc, 46fc, error 0x8c). `&7==2` or `&0x18`: TX/cal busy (6c92). Bits 15:8: error cause bytes (IRQ8 E0.1 handler reads `>>8`) | OBS |
| `R008` | RX PHR length (`&0x7f`, includes FCS) at frame end | OBS 5370 |
| `R00c` | RSSI of last frame: bits 11:4 raw; dBm = raw - `rssi_offset` (clamped >= -127) | OBS 5370, 4ca8(3) |
| `R048` | bit15 = **TX FIFO busy** (writeTxFifo error 0x84) | OBS 45fc, PHY 4458 |
| `R050` | received FCF (16 bit), valid at E0.3 | OBS 5370 |
| `R054` | received sequence number, valid at E0.3 | OBS 5370 |
| `R078`,`R07c` | ED/RSSI results; `R07c` = energy-scan result read after scan (openthread_radio `a500`->4ca8(2)); `R078` selected by arg 1 (unused here) | OBS/INF |
| `R080` | instantaneous RSSI: bit15 = valid (polled <= 8x16 us), bits 11:4 raw (4ca8(0), `otPlatRadioGetRssi` a064) | OBS |
| `R090`,`R094` | time counter: low 16 / bits 23:16 (4de2 reads **low first, then high**, IRQs off) | OBS |
| `R0a4` | written once at init from `bbea(1)` (a334 -> 5114): 16-bit value; copied into a security counter seed (70ac) | OBS, purpose INF |
| `R0b4`,`R0b8` | **event (raw pending) flags** E0 / E1 (16 bits each, 32 combined). Cleared by writing 0 (init) or W1C at `R8b4/R8b8` (52c0) | OBS |
| `R0ec` | misc control bits; only touched through SET/CLR aliases: init clear `0xff`, `CLR 1`,`SET 2`; timer arm `CLR 8, SET 4` (4384); teardown `CLR 3`,`CLR 0xc`, `CLR 0x4f` (4d50, 3f30). Never read | OBS writes, meaning unknown |
| `R0f8` | init `= 0xc0` | OBS |
| `R0fc` | PHY only: SET/CLR `0x40` (debug pin) | OBS PHY 5376 |
| `R100` | **MAC control** (read-mod-write and SET/CLR). bit0 = RX enable flag; bit15 = hardware appends FCS (writeTxFifo sends `len-2` bytes when set, 45fc); bits 14:13 = mode field, init `(old&0x9fff)|0x2000` ("configMac2015NonBeaconed" string); bit12 cleared at init; bits 7:6 `0x40`/0 written by 496c (candidate: pending-frame/CSMA related, unresolved); bit1 SET at init; bit10 cleared (`R900=0x400`) | OBS fields, names INF |
| `R104` | PAN id (`PANID=%X` fn 9952 -> 49b8) | OBS |
| `R108` | short address (`Short_Addr=%X` 9978 -> 49aa; `0xfffe` = none) | OBS |
| `R10c..R118` | extended address, 4 x 16-bit words, LE byte order (49c6/4a1c) | OBS |
| `R130` bits1:0, `R138` bits11:0 | address-filter enable fields. Set by 48bc(1) (value from RAM bytes, default effectively 0), cleared by 48bc(0) = promiscuous (`otPlatRadioSetPromiscuous` a0b8) | OBS writes, meaning INF |
| `R184` | init `= 0x1ff` | OBS |
| `R188` | init `= 0` | OBS |
| `R18c` | `((0x76-x)|(0xc0-x)<<8)&0xffff`, `x` from RF reg 0x0e (3794); RX handler writes `0` after queuing an ACK | OBS values, INF timing constant |
| `R194` | CCA: bits7:0 threshold clamped `0x26..0x32`, bits13:8 `0x2f`, bits15:14 mode `1` (47f8, 4828; PHY CLI "CCA threshold") | OBS fields, INF names |
| `R198`,`R19c` | counter modulus: low 16 = `0x322e`, high = `0xab` (period `0xab322e`) | OBS 3794 |
| `R1a8,R1b0,R1b8,R1c0` | four 24-bit **compare** values (low 16 at base, bits 23:16 at base+4): C0, C1, C2, C3 (4e74, 4f00, 4f5c, 4fc4) | OBS |
| `R1c8,R1cc` | IRQ8 enable masks for E0, E1 | INF (see 3.2) |
| `R1d0,R1d4` | IRQ22 enable masks for E0, E1 | INF |
| `R1d8` | third mask pair, written 0 (INF: unused) | OBS |
| `R1f0` | init `= 0x5f80` | OBS |
| `R3e8` | **RX FIFO port**: each 32-bit read pops one byte (low 8 bits) (52e6, 5370) | OBS |
| `R3ec` | bit7 polled == 1 at init (up to 1 s, error 0x82); INF: PHY/FIFO ready | OBS |
| `R3f8`,`R3fc` | **command port**, see 2.2 | OBS |
| `R500` / `R900` | SET / CLR alias of `R100` | INF |
| `R5d4`,`R9d4` | SET / CLR of `R1d4` | INF |
| `R9cc`,`R5cc` | CLR / SET of `R1cc` | INF |
| `R8b4`,`R8b8` | W1C ack of E0 / E1 | OBS |

### 2.2 Command port (`R3f8` arg, `R3fc` opcode)

Writes to `0x600013fc` are commands: low byte = opcode, high bytes = argument
(a second word may be pre-written to `0x600013f8`).

| Opcode | Use | Tag |
| --- | --- | --- |
| `0x16` | push **TX FIFO** byte: word = `(byte << 8) | 0x16`. First push = PHR (PSDU length incl. FCS, <= 0x7f), then `len` bytes (`len-2` if `R100.15`) (519c) | OBS |
| `0x04` | transmit now (5128 with delay 0) | OBS; "now" INF |
| `0x06` | transmit after delay: `R3f8 = delay`, then `R3fc = 6` (5128). Delay in 16 us ticks, <= 0xffff, relative to command time. ACK TX uses 7 (hdr < 15) or 8 | OBS, unit INF |
| `0x05` / `0x07` | same as 4 / 6, chosen when a state flag at lld+0x2c6 is set (515e); never set in code seen. PHY text: "CSMA-CA mechanism is enabled. Enabling it will disable Tx CCA" | OBS/INF CSMA variant |
| `0x0b` | start energy detect (4d36, energy scan a1f0) | OBS |
| `0x0c` | arg `(R3f8 = arg&0xffff, R3fc = ((arg>>8)&0xff00)|0xc)`; init calls with 2 (4e1c) | OBS; INF timing parameter |
| `0x14` | abort sequencer after an error event (IRQ22 bits 8,9,10 and E1 11,12) | OBS |
| `0x15` | release RX FIFO / drop frame (error and filtered paths in 5370). Success path does not issue it | OBS |
| `0x03` | after RX RF-calibration retry, if state 3/5 (6c92) | OBS, meaning INF (re-arm) |

### 2.3 Bring-up (what a firmware must do, in order)

1. Precondition: `RCC_CR` bit17 (HSERDY) = 1, else lld error 0x97 (3f30, 3794).
2. Clocks (3f30): `RCC+0x44 |= 1`; `RCC_C2APB3ENR (0x58000164) |= 1`, then `|= 2`
   (read back each); `RCC+0x44 &= ~1`; wait `RCC+0x94` bit16 == 0 (<= 10 ms,
   10 us steps). INF: `+0x44` is `RCC_APB3RSTR`, `+0x94` bit16 is a radio
   reset/busy status; names unverified. The firmware performs this reset cycle
   twice (before and after a deinit); once is enough for a model.
3. Power the RF (7cd4(1,1) / 7d4c): `0x60000404 = 3`, then probe `0x6000000c`
   until `0xa5a5` reads back (<= 1000 tries). Then in 7d4c:
   `0x60000418 |= 2` (plus `|= 8` if bits 13:8 nonzero), `0x60000418 &= 0xffffc0f7`.
4. RF register init via the descriptor bus (7d4c, see table 1.3): `0x2d=0x60`;
   `0x05 |= 6`; `0x06 |= 1`; `0x41 = 0x80|(old&~0x40)`; 21-byte table at `0x65`;
   `0x39=0`; `0x0e = 2|(old&~0x1e)`; OTP trims (82bc).
5. TX power (803c, power 0 index) and channel (7fc0) via RF regs 5 and 4.
6. NVIC: priority 2 and enable IRQ30. Wait RF state (`0x5c&0x1f`) == 4 (1 s).
7. MAC init (3794): `R0b4 = R0b8 = 0` (mask all), `R0f8 = 0xc0`, `R1e8 = 0x87d`
   (INF: part of the timing block; value from literal 0x87d), `R8ec = 0xff`
   (via `0x600018ec`), PWR_SCR (`0x58000418`) `= 0x400`, poll `R3ec` bit7.
   `R188 = 0`, CCA (`R194`: threshold, `0x2f`, mode 1), `R184 = 0x1ff`, masks:
   `R1c8 = 0x084d`, `R1cc = 0x8600`, `R1d0 = 0x1700`, `R1d4 = 0x7801`, `R1d8 = 0`
   (51ec), `R1f0 = 0x5f80`, `R900 = 0x400`.
8. Timer: `R198 = 0x322e`, `R19c = 0xab`; zero compares (`R1a8,R1b0,R1b8,R1c0`).
   `0x600019cc = 0x40`, `0x600018ec = 1`, `0x600014ec = 2`, 100 us delay,
   `R0b4 = R0b8 = 0`, PWR_SCR = `0x400`, `R900 = 1`, `R100 &= 0xff3f`,
   `R900 = 4`, `R100 = (R100 & 0x9fff) | 0x2000`, `R100 &= 0xefff`,
   `R500 = 2`, `R500 = 0x8000` (enables HW FCS), read RF reg 0x0e -> `R18c`.
   `R138 &= 0xf000`, `R130 &= 0xfffc` (filters off), `R0b4 = R0b8 = 0`.
9. PWR: `PWR_SCR = 0x400`; set bit10 in `0x58000484` (INF `PWR_C2CR3`) and poll it
   reads back set (3794). NVIC priorities/enables: IRQ8 prio 1, IRQ22 prio 2,
   IRQ31 prio 1, all enabled.
10. Put the radio in RX: set `R500 = 1`, poll `R000&0x1f == 0` first, then RF state
    must reach `0x1a` within 20 ms (4250(1), end of 3794).

Error codes seen: 0x97 no HSE, 0x9c MAC busy timeout (4200), 0x9d unexpected RF
state, 0x82 FIFO ready timeout, 0x92 not initialised, 0x65/0x61/0x62/0x66 RF bus.

### 2.4 Set channel (368c) and default state (4250)

- Channel 11..26 else error 2. Wait `R000&0x1f == 0` (12 ms). Read `R100`.
  If bit0 set: write `R900 = 1`, poll RF reg `0x5c&0x1f == 4`; else require it
  is 4. Write RF reg 4 = `(ch-11)*2` (skipped if unchanged), 100 us delay,
  then restore RX with 4250(old bit0).
- `4250(0)` (idle): if `R100.0`: `R900 = 1`, wait RF state 4 (12 ms); else RF state
  must already be 4. `4250(1)` (RX): if not `R100.0`: `R500 = 1`, wait RF
  state `0x1a` (20 ms); else it must already be `0x1a`.

### 2.5 TX power (803c)

Valid -21..+6 dBm, else error 2. Writes RF reg 5 bits7:3 (mask `0xf8`) with
`table[dBm+22] << 3` (skipped if unchanged). The table is in flash at
`0x14a13`: `00 01 02 03 04 05 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 14 16
19 1a 1b 1c 1d 1e 1f` (index 0 = 00, then entries for -21..+6). INF: located by
pattern, copied to RAM before use; the monotone shape and length (28 values for
28 dBm steps) make the mapping very likely.

### 2.6 Receive path (IRQ8, fn 5370)

Frame events in order, all via IRQ8:
1. **E1.15** (`0x80000000`): SFD / frame start. Handler stamps `now` (software
   timestamp, no hardware capture register seen) into the frame meta. INF.
2. **E0.3**: header decoded. Firmware reads `R050` (FCF), `R054` (sequence),
   parses addressing from the FCF (7758), copies PAN/short/ext address from
   `R104/R108/R10c..R118`, and for AR frames clears `R100` bits 14:13.
   Software then checks the destination PAN / short / ext address itself
   (5d..5f paths), so a model that delivers every frame still works.
3. **E0.0**: frame end. `len = R008 & 0x7f` (includes FCS). Firmware pops
   the header (`hdr` bytes from fn 7758) from `R3e8`, then the rest; total =
   PHR length. RSSI = `(R00c & 0xff0) >> 4` minus offset; LQI is computed in
   software from RSSI (soft-float, constants 97.0 and 255.0). FCS is checked by
   the hardware: a bad FCS never reaches E0.0 (INF: appears as E0.1, whose handler
   releases the FIFO with opcode `0x15`).
4. Valid RX of an ACK cancels the ACK timeout: `R9d4 = 0x10`.

ACK generation is **software**: for an AR frame the handler builds the ACK
(6e0c; FCF byte0 `0x02`, `0x12` if frame pending from the source-match table;
enhanced ACK for 2015 frames), pushes it with the TX FIFO commands, and sends it
with `R3f8 = 7|8, R3fc = 6` (delayed TX, delay counted from the command, so
software latency makes up the 192 us turnaround). So hardware has no auto-ACK
and no source-match table; both are software (RAM tables at 4a68/4b84).
INF for "no HW auto-ACK": the handler queues the ACK explicitly and no register
enable for auto-ACK was found. The PHY text mentions "txAutoAck" as an lld
field, so a hardware knob may exist unused by the Thread image.

### 2.7 Transmit path

`writeTxFifo` (45fc): `len <= 0x7f` (error 0x91), wait `R000&7 == 0` (error
0x8c), `R048.15 == 0` (error 0x84), push PHR then bytes with opcode `0x16`.
`transmitPreviousPacket` (46fc): requires a loaded FIFO (0x85), `R000&7 == 0`
(0x8c); computes delay = (earliest TX time - now) in ticks, 0 if past, 0x21 if
> 0xffff; sets `R3f8 = delay`, `R3fc = 6` (or `R3fc = 4` if delay 0).
Earliest TX time = last RX/TX event time + IFS (5 ticks if PHR < 0x13, else
0x21 ticks; 5320, 45fc). The radio is expected to be in RX when TX is issued
(`otPlatRadioTransmit` does set channel then `4250(1)` first); INF: the
sequencer returns to RX after TX because `R100.0` stays set. CSMA is software
or the opcode 5/7 variants; the Thread image uses 4/6.

TX events (IRQ22): **E1.0** SFD sent: timestamp `now+10`. **E0.12** TX done:
without ACK request -> success; with AR (state 3): RX stays on and the ACK
timeout is armed: `R8b8 = 0x10` (clear pending E1.4), compare C0 = counter +
`0x36` ticks (54 symbols, `0x68` for enhanced ACK), `R5d4 = 0x10` (enable
compare-0 IRQ in IRQ22). **E1.4** (compare C0 reached): `R9d4 = 0x10`, result 0x40 (no ACK).
Errors: E0.8/E0.9 -> `0x14`, result 0x8d; E0.10 -> `0x14`, result 0x89 (or 0x83
if a flag is set; INF channel access failure); E1.11/12/13/14 -> results
0x87/0x88/0x86/0x8c (E1.11/12 also `0x14`). Exact causes are INF.

### 2.8 Energy detect / CCA / RSSI

- Instantaneous RSSI: poll `R080` bit15, dBm = `((R080>>4)&0xff) - offset`.
- Energy scan (a1f0): set channel, `4250(1)`, `R3fc = 0x0b`, set compare C1
  (`R1b0`) = now + `ms*1000/16` ticks, enable E1.5 (`R5d4 = 0x20`). At E1.5
  read `R07c`, disable (`R9d4 = 0x20`). Result is `((R07c>>4)&0xff) - offset`.
- `rssi_offset` (RAM short at `0x2003cac0`) has no initialiser in the decompile
  (scatter-loaded data). Model: any constant; firmware just subtracts it.
- CCA threshold: `R194[7:0]` clamped `0x26..0x32`; CCA/CSMA hardware behaviour for
  opcodes 5/7 is unverified.

## 3. Interrupts

### 3.1 NVIC lines (OBS vector table @0, handlers in startup_and_isr)

| IRQ | Handler fn | Role |
| --- | --- | --- |
| 8 | 5370 | RX events (and compare C2 if enabled) |
| 22 | 61e4 | TX events and compare events C0, C1, C3 |
| 30 | 81fc | PHY / radio-controller events (`0x60000418`, cause in RF reg 0x5d), 32 kHz calibration |
| 31 | 6a58 | counter wrap (see 3.3); flag is `PWR_SR1` (`0x58000410`) bit10 |

All other CPU2 vectors spin. Priorities: 8 -> 1, 22 -> 2, 30 -> 2, 31 -> 1 (8974).

### 3.2 Event flags and masks (INF structure, OBS values)

`E0 = R0b4`, `E1 = R0b8`; a combined word `ev = (E0 & mask0) | ((E1 & mask1) << 16)` is
formed by every handler (loops until stable, then `W1C` ack via `R8b4/R8b8`).
Handler line, mask pair: IRQ8 uses `R1c8`/`R1cc`, IRQ22 uses `R1d0`/`R1d4`. The
initial mask values (3.2 table) agree exactly with the bits each handler tests.

| Event | IRQ | Default mask | Handler action (OBS) |
| --- | --- | --- | --- |
| E0.0 | 8 | `R1c8` bit0 | RX frame complete (2.6) |
| E0.1 | 8 | not in default mask | RX error/reject: read `R000>>8`, `R3fc=0x15`, cb |
| E0.2 | 8 | `R1c8` bit2 | cb (+0x24), meaning unknown |
| E0.3 | 8 | `R1c8` bit3 | RX header/FCF available |
| E0.6 | 8 | `R1c8` bit6 | RX secondary completion for AR frames, cb +0x2c |
| E0.11 | 8 | `R1c8` bit11 | cb +0x30, unknown |
| E1.6 | 8 | enabled on demand (`R5cc`) | compare C2 event |
| E1.9, E1.10 | 8 | `R1cc` | cb +0x1c, unknown |
| E1.15 | 8 | `R1cc` bit15 | SFD / RX start timestamp |
| E0.8/9/10 | 22 | `R1d0` | TX error group (above) |
| E0.12 | 22 | `R1d0` bit12 | TX done |
| E1.0 | 22 | `R1d4` bit0 | TX SFD sent, stamp `now+10` |
| E1.4 | 22 | on demand | compare C0 (ACK timeout) |
| E1.5 | 22 | on demand | compare C1 (energy-scan end) |
| E1.7 | 22 | on demand | compare C3 (software alarm, cb +0x6c) |
| E1.11..14 | 22 | `R1d4` | TX/RX error results (above) |

Compare-event mapping: C0->E1.4, C1->E1.5, C2->E1.6, C3->E1.7 (consistent
with each compare being armed together with its enable bit: 4384 for C3,
a1f0 for C1, the TX-done path for C0). Mask enable/disable is done through the
SET/CLR aliases of `R1d4` (`R5d4`/`R9d4`) and `R1cc` (`R5cc`/`R9cc`).

### 3.3 Time base (OBS 4de2, 5024, 50fa, 4384)

- Counter: 24 bits, `R090` low 16 and `R094` bits 7:0. One tick = 1 symbol =
  **16 us** (62.5 kHz): energy-scan duration uses `ms*1000>>4` (a1f0), `50fa`
  returns `ticks << 4` as microseconds, ACK wait is 54 ticks = 864 us.
  Wraps at the programmed modulus (`R198/R19c` = `0xab322e`, about 179.5 s).
- 64-bit time (5024) = software accumulator + counter; the accumulator
  (`lld+0x2e8`) is advanced by IRQ31 each wrap (`+= modulus`, 6a58), and
  5024 detects a not-yet-serviced wrap itself.
- Compare registers `R1a8,R1b0,R1b8,R1c0`: 24-bit values; equal to the counter
  -> event E1.4..7 (if its mask bit is set the IRQ fires). A value of 0 is "off".
- Wrap IRQ: `PWR_SR1` bit10 set by hardware, ack by `PWR_SCR = 0x400`; enable
  is bit10 of `0x58000484` (INF). Modelling the wrap is optional below ~179 s.

### 3.4 `0x60000418` interrupt register (OBS 81fc, 7f44, 7f74, 7f9c; PHY 7fce)

Two (status, enable) pairs: bit0 status A / bit1 enable A (slow-clock
event, calls the Clk32 routine); bit2 status B / bit3 enable B (PHY event; cause
in RF reg 0x5d). Status bits are cleared by **writing 1 to them**: the ack helper
writes `(x & ~4) | 1` for A and `(x & ~1) | 4` for B. Bits 13:8 are per-cause enables
for event B; setting any of them also sets bit3. Bring-up leaves them clear and
sets bit1 (7d4c).

## 4. Minimum viable emulator subset

Enough for a simple RCP (set channel, RX, TX with ACK wait, software
filtering) and for the ST lld image itself to boot:

1. **`0x6000000c`**: 16-bit register; probe retention rule (1.2); descriptor
   command executed instantly (clear bit16) against a 125-byte RF file; reads
   return the file. RF reg `0x5c` = `0x1a` if `R100.0`, else `0x04`; reg `0x5d` = 0.
2. **`0x60000404`** and **`0x60000418`**: plain RW with W1C on bits 0 and 2 of
   `0x60000418`. IRQ30 never needs to fire for basic operation.
3. **MAC window** `0x60001000..0x600013ff` as 16-bit registers with `+0x400` W1S
   and `+0x800` W1C aliases (only offsets `0x000..0x1ff` need aliases).
   Reads return what was written, except the registers below.
4. Dynamic registers: `R000` = 0 idle (nonzero while a TX is in air); `R048` bit15 = 0
   unless TX FIFO busy; `R3ec` = `0x80`; `R008`, `R00c`, `R050`, `R054` from the RX
   frame; `R080` = `0x8000 | rssi<<4`; `R07c` = ED; `R090/R094` = counter.
5. **Counter and compares** at 62.5 kHz of simulated time with the modulus in
   `R198/R19c`. Implement C0 (ACK timeout), C1 (ED) and C3 (alarm) so that
   `ev & mask` raises IRQ22.
6. **E0/E1 raw flags, masks and W1C**, plus IRQ8/IRQ22 lines. RX: E1.15 at
   SFD, E0.3 after the MHR (or at SFD+header time), E0.0 at frame end; FIFO pop
   port `R3e8` (one byte per read), full PHR length available.
7. **Command port**: opcode `0x16` pushes TX FIFO, `4/6` (and `5/7`) start TX
   (delay `R3f8`), `0x0b` start ED, `0x14` abort, `0x15` flush RX FIFO. TX: at
   start E1.0, at end E0.12; deliver to the shared air; hardware does not generate
   ACKs, the peer's ACK arrives as an ordinary RX.
8. **RCC/PWR minimum**: `RCC_CR.HSERDY=1`, `RCC+0x94` bit16 reads 0,
   `RCC_C2APB3ENR` bits0/1 writable, PWR_SR1/SCR/`0x58000484` bit10 writable.

Can be stubbed (read as written or zero): all RF registers except 4/5/0x5c/0x5d;
`R0ec`, `R0f8`, `R184`, `R188`, `R18c`, `R194`, `R1e8`, `R1f0`, `R100` bits other than
0 and 15, filter fields `R130/R138`, `R3fc` opcodes `0x03/0x0c`, Clk32 regs,
`0x60000400/408/40c/410/414`, OTP trim at `0x1fff7640`, the counter wrap
interrupt, calibration events, hardware CSMA/CCA (use software CSMA or no CCA),
RSSI offset (choose a constant), LQI (software).

Semantics that must be right: TX FIFO word format (`(b<<8)|0x16`, first word is
PHR), HW FCS (`R100.15`: firmware sends `len-2` bytes and expects FCS appended and
checked), delay unit 16 us relative to command, `R3f8` delay is consumed by
opcode 6, 64-bit time via counter.

## 5. Unknowns

- Exact meaning of E0.2, E0.6, E0.11, E1.9/10 and of the result codes 0x83, 0x86..0x8d.
- Whether the mask registers `R1c8..R1d8` are indeed enables (strong structural
  match with handler tests; the original names are unknown) and the third pair
  `R1d8` (IRQ31 path? it reads PWR instead).
- Behaviour of `R0ec` bits and of opcodes `0x03`, `0x0c`, `0x05`, `0x07` (CSMA variants).
- `R100` bits 7:6 and bits 14:13 semantics; `R130/R138` meaning and defaults
  (loaded from RAM bytes at `lld+0x2cb/0x2cc` not initialised in decompiled code).
- Hardware address filtering behaviour (whether the radio drops non-matching
  frames before E0.3); the firmware re-checks addresses in software anyway.
- RSSI offset value, ED/RSSI scaling beyond "bits 11:4 minus offset", `R078`.
- Units/timing of the ACK delay (7/8) from `R3f8`, whether TX returns to RX
  automatically (INF yes), and the FIFO depth (writes up to 0x7f bytes accepted).
- RCC+0x44 / RCC+0x94 bit16 / `0x58000484` register identity, and the RM0434 names
  for CPU2 IRQ 8, 22, 30, 31 (not checked against the reference manual).
- PHY mode values 0x2000/0x2001/0x2003 (Thread uses 0x2001) and RF registers
  other than the ones listed.
- Whether `R3ec` bit7 is FIFO-ready or PHY-ready; effect of RF reg 0x0e on `R18c`.
- Slow-clock (Clk32) calibration and sleep/wake (`enterSleep`/`exitSleep`) are
  not documented here.
