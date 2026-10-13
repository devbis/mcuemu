#pragma once

#include <iosfwd>

namespace sim {

// `mcuemu sim ...`; argv[1] is the subcommand name. Throws on bad input.
int sim_command(int argc, char **argv);
void print_sim_usage(std::ostream &out);
// `mcuemu serve ...`: the JSON-RPC/WebSocket control server.
int serve_command(int argc, char **argv);
void print_serve_usage(std::ostream &out);

} // namespace sim
