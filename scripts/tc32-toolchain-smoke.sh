#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
toolchain_bin="$repo_root/../llvm-tc32-arm-based/toolchains/tc32-stage2/llvm/bin"
clang_path="${TC32_CLANG:-$toolchain_bin/clang}"
objcopy_path="${TC32_OBJCOPY:-$toolchain_bin/llvm-objcopy}"
build_dir="$repo_root/build"

if [[ ! -x "$clang_path" || ! -x "$objcopy_path" ]]; then
    echo "Set TC32_CLANG and TC32_OBJCOPY to a TC32-capable LLVM toolchain." >&2
    exit 2
fi

cmake -S "$repo_root" -B "$build_dir" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$build_dir"
"$clang_path" -target tc32-unknown-none-elf -c \
    "$repo_root/tests/tc32/isa-smoke.s" -o "$build_dir/isa-smoke.o"
"$objcopy_path" -O binary --only-section=.text \
    "$build_dir/isa-smoke.o" "$build_dir/isa-smoke.bin"

test_output="$("$build_dir/tc32-emu" run --model TLSR8258F512 \
    --firmware "$build_dir/isa-smoke.bin" --steps 20)"
printf '%s\n' "$test_output"

[[ "$test_output" == *"r0=0x7"* ]]
[[ "$test_output" == *"r4=0xa"* ]]
[[ "$test_output" == *"r5=0x8"* ]]
[[ "$test_output" == *"r6=0x8"* ]]
[[ "$test_output" == *"r2=0x2a"* ]]
[[ "$test_output" == *"r3=0x1"* ]]
[[ "$test_output" == *"stopped on tserv software-service trap"* ]]
