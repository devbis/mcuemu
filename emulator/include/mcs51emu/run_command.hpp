#pragma once

namespace mcs51emu {

// `mcuemu run --model CA51F253L3 ...`: runs one image on one core and prints
// the final state. `argv[0]` is the program and `argv[1]` is "run", as in main.
int run_command(int argc, char **argv);

} // namespace mcs51emu
