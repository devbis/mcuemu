#pragma once

namespace nrf52emu {

// `mcuemu run --model NRF52840 ...`: runs one image on one chip and prints the
// final state. `argv[0]` is the program and `argv[1]` is "run", as in main.
int run_command(int argc, char **argv);

} // namespace nrf52emu
