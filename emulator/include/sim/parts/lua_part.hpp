#pragma once

#include "sim/part.hpp"

#include <memory>

namespace sim {

// Part type "script": the behavior is a Lua 5.4 file (see docs/scripting.md).
std::unique_ptr<Part> make_lua_part(PartContext &context);

} // namespace sim
