#pragma once

namespace esp32emu {

// `mcuemu run --model ESP32C6 ...`: runs one image on one chip and prints the
// final state. `argv[0]` is the program and `argv[1]` is "run", as in main.
int run_command(int argc, char **argv);

} // namespace esp32emu
