#pragma once

namespace efr32emu {

// `mcuemu run --model EFR32MG21 ...`: runs one image on one chip and prints the
// final state. `argv[0]` is the program and `argv[1]` is "run", as in main.
int run_command(int argc, char **argv);

} // namespace efr32emu
