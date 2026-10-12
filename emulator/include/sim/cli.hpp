#pragma once

#include <iosfwd>

namespace sim {

// `tc32-emu sim ...`; argv[1] is the subcommand name. Throws on bad input.
int sim_command(int argc, char **argv);
void print_sim_usage(std::ostream &out);

} // namespace sim
