#!/usr/bin/env bash
# Builds tests/tlsr826x/main.c against the Telink Zigbee SDK startup and the
# 826x driver library with the TC32 LLVM toolchain. Output: $OUT/tlsr826x-smoke.bin
# (never commit it). Needs TELINK_SDK (tl_zigbee_sdk) and TC32_BIN (llvm/bin).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
sdk="${TELINK_SDK:-$repo/../telink/telink_zigbee_sdk/tl_zigbee_sdk}"
bin="${TC32_BIN:-$repo/../telink/llvm-tc32-arm-based/toolchains/tc32-stage2/llvm/bin}"
out="${OUT:-$repo/build/tlsr826x}"
mkdir -p "$out"
cc=(-target tc32-unknown-none-elf -Os -ffunction-sections -fdata-sections -fno-builtin
    -DMCU_CORE_826x -DCHIP_8269 -I"$here" -I"$sdk/platform/chip_826x" -I"$sdk/platform" -I"$sdk/proj/common" -I"$sdk/proj")
# cstartup uses TC32 status-register moves (tmrss). This LLVM build encodes the
# SDK's `tmrcs` as `tmrss` (0x6bdb instead of 0x6bcb) in other files, so assemble
# the startup with the vendor gcc 4.5 toolchain when it is present.
gcc="${TC32_GCC:-$repo/../telink/tc32/bin/tc32-elf-gcc}"
asm_flags=(-DMCU_CORE_826x -DCHIP_8269 -I"$here")
if [[ -x "$gcc" ]]; then
    "$gcc" "${asm_flags[@]}" -c "$sdk/platform/boot/826x/cstartup_826x.S" -o "$out/cstartup.o"
else
    "$bin/clang" "${cc[@]}" -c "$sdk/platform/boot/826x/cstartup_826x.S" -o "$out/cstartup.o"
fi
"$bin/clang" "${cc[@]}" -c "$here/divmod.c" -o "$out/divmod.o"
"$bin/clang" "${cc[@]}" -c "$here/main.c" -o "$out/main.o"
# ld.lld wants absolute operands where GNU ld did not; wrap the two offending sums.
python3 - "$sdk/platform/boot/826x/boot_826x.link" "$out/boot_826x.link" <<'PY'
import sys
s = open(sys.argv[1]).read()
s = s.replace("AT ( _dstored_bin_ + _end_data_ - _start_data_ )", "AT ( _dstored_bin_ + (_end_data_ - _start_data_) )")
s = s.replace("PROVIDE(_bin_size_ = _code_size_ + _end_data_ - _start_data_ + SIZEOF(.sdk_version));", "PROVIDE(_bin_size_ = _code_size_ + (_end_data_ - _start_data_) + SIZEOF(.sdk_version));")
open(sys.argv[2], "w").write(s)
PY
"$bin/ld.lld" -T "$out/boot_826x.link" \
    --defsym=__BOOT_LOADER_IMAGE=0 --defsym=__FW_RAMCODE_SIZE_MAX=0x0 --defsym=__FW_OFFSET=0 \
    --gc-sections -o "$out/tlsr826x-smoke.elf" "$out/cstartup.o" "$out/main.o" "$out/divmod.o" \
    "$sdk/platform/lib/libdrivers_826x.a"
"$bin/llvm-objcopy" -O binary "$out/tlsr826x-smoke.elf" "$out/tlsr826x-smoke.bin"
echo "$out/tlsr826x-smoke.bin"
