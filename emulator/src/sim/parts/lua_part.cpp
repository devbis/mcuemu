#include "sim/parts/lua_part.hpp"

#include "sim/parts/i2c_slave.hpp"

extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}

#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace sim {

namespace {

constexpr std::size_t MemoryLimitBytes = 64u << 20;
constexpr int HookInterval = 10000;
constexpr int HookLimit = 1000; // 10M VM instructions per callback
constexpr std::size_t EdgeHistory = 64;
constexpr const char *PinMeta = "tc32emu.pin";

struct Allocator {
    std::size_t used{0};
};

void *limited_alloc(void *ud, void *ptr, std::size_t osize, std::size_t nsize) {
    auto *a = static_cast<Allocator *>(ud);
    const std::size_t old = ptr ? osize : 0;
    if (nsize == 0) {
        if (ptr) {
            a->used -= old;
            std::free(ptr);
        }
        return nullptr;
    }
    if (a->used - old + nsize > MemoryLimitBytes)
        return nullptr;
    void *fresh = std::realloc(ptr, nsize);
    if (fresh)
        a->used += nsize - old;
    return fresh;
}

} // namespace

class LuaPart final : public Part {
public:
    explicit LuaPart(PartContext &ctx);
    ~LuaPart() override;

    std::string type() const override { return "script"; }
    void register_endpoints(EndpointRegistry &registry,
                            const std::function<std::uint64_t()> &now) override;

private:
    struct PinSlot {
        std::string name;
        Pin *pin;
        std::deque<std::pair<std::uint64_t, bool>> edges; // (time, high)
    };
    struct PinHandle {
        LuaPart *part;
        std::size_t index;
    };
    struct Timer {
        EventId event;
        int ref;
    };
    struct Exposed {
        std::string name;
        int getter_ref;
        int setter_ref; // LUA_NOREF when read-only
    };

    static LuaPart *self(lua_State *L) {
        return *static_cast<LuaPart **>(lua_getextraspace(L));
    }
    static int traceback(lua_State *L);
    static void hook(lua_State *L, lua_Debug *);

    // Lua-facing functions.
    static int l_now(lua_State *L);
    static int l_at(lua_State *L);
    static int l_after(lua_State *L);
    static int l_cancel(lua_State *L);
    static int l_expose(lua_State *L);
    static int l_i2c_slave(lua_State *L);
    static int l_rand(lua_State *L);
    static int l_log(lua_State *L);
    static int pin_drive(lua_State *L);
    static int pin_release(lua_State *L);
    static int pin_level(lua_State *L);
    static int pin_level_at(lua_State *L);
    static int pin_mv(lua_State *L);
    static int pin_set_mv(lua_State *L);
    static int pin_name(lua_State *L);
    static int radio_transmit(lua_State *L);
    static int radio_listen(lua_State *L);
    static int radio_set_power(lua_State *L);
    static int radio_airtime(lua_State *L);
    void on_air_frame(const tc32emu::RadioFrame &frame);
    void deliver_frame(tc32emu::RadioProtocol protocol, std::uint8_t channel,
                       std::uint64_t end_ns);

    static PinSlot &slot_of(lua_State *L, int index);
    // `observer` calls (endpoint get/set) still run after the script stopped.
    bool call(int nargs, int nresults, bool observer = false);
    void report(const std::string &message);
    void on_net_change(std::size_t index, const NetChange &change);
    void fire_timer(std::uint64_t id);
    std::size_t terminal_index(const std::string &name) const;
    void push_params(const std::map<std::string, std::string> &params);
    void push_value(const Value &value);
    Value to_value(int index);
    int ref_function_field(int table_index, const char *field);
    bool call_ref_bool(int ref, bool fallback, int nargs_pushed_below);

    Scheduler &scheduler_;
    Allocator allocator_;
    lua_State *L_{nullptr};
    std::vector<PinSlot> pins_;
    std::map<std::uint64_t, Timer> timers_;
    std::uint64_t next_timer_{1};
    std::vector<Exposed> exposed_;
    std::vector<std::unique_ptr<I2cSlave>> i2c_slaves_;
    std::vector<int> owned_refs_;
    std::uint64_t current_ns_{0};
    std::uint64_t rng_;
    int hook_count_{0};
    bool in_own_drive_{false};
    bool failed_{false};
    // Radio (only when the part has a `radio:` section).
    tc32emu::VirtualAir *air_{nullptr};
    std::string radio_node_;
    std::vector<std::string> radio_protocols_;
    bool listening_{false};
    tc32emu::RadioProtocol listen_protocol_{tc32emu::RadioProtocol::Zigbee802154};
    std::uint8_t listen_channel_{0};
};

int LuaPart::traceback(lua_State *L) {
    const char *message = lua_tostring(L, 1);
    luaL_traceback(L, L, message ? message : "(error object is not a string)", 1);
    return 1;
}

void LuaPart::hook(lua_State *L, lua_Debug *) {
    LuaPart *part = self(L);
    if (++part->hook_count_ > HookLimit)
        luaL_error(L, "script exceeded %d million instructions in one callback",
                   HookLimit * HookInterval / 1000000);
}

void LuaPart::report(const std::string &message) {
    std::cerr << "error: part '" << name() << "': " << message
              << "\n(the script is stopped)\n";
    failed_ = true;
}

bool LuaPart::call(int nargs, int nresults, bool observer) {
    if (failed_ && !observer) {
        lua_pop(L_, nargs + 1);
        return false;
    }
    hook_count_ = 0;
    const int base = lua_gettop(L_) - nargs;
    lua_pushcfunction(L_, traceback);
    lua_insert(L_, base);
    const int status = lua_pcall(L_, nargs, nresults, base);
    lua_remove(L_, base);
    if (status != LUA_OK) {
        const char *message = lua_tostring(L_, -1);
        if (!failed_)
            report(message ? message : "unknown Lua error");
        lua_pop(L_, 1);
        return false;
    }
    return true;
}

LuaPart::PinSlot &LuaPart::slot_of(lua_State *L, int index) {
    auto *handle = static_cast<PinHandle *>(luaL_checkudata(L, index, PinMeta));
    return handle->part->pins_[handle->index];
}

std::size_t LuaPart::terminal_index(const std::string &terminal) const {
    for (std::size_t i = 0; i < pins_.size(); ++i)
        if (pins_[i].name == terminal)
            return i;
    throw std::invalid_argument("terminal '" + terminal + "' is not wired");
}

void LuaPart::push_value(const Value &value) {
    if (const auto *b = std::get_if<bool>(&value))
        lua_pushboolean(L_, *b);
    else if (const auto *i = std::get_if<std::int64_t>(&value))
        lua_pushinteger(L_, static_cast<lua_Integer>(*i));
    else if (const auto *d = std::get_if<double>(&value))
        lua_pushnumber(L_, *d);
    else if (const auto *s = std::get_if<std::string>(&value))
        lua_pushlstring(L_, s->data(), s->size());
    else
        lua_pushnil(L_);
}

Value LuaPart::to_value(int index) {
    switch (lua_type(L_, index)) {
    case LUA_TBOOLEAN: return Value{static_cast<bool>(lua_toboolean(L_, index))};
    case LUA_TNUMBER:
        if (lua_isinteger(L_, index))
            return Value{static_cast<std::int64_t>(lua_tointeger(L_, index))};
        return Value{static_cast<double>(lua_tonumber(L_, index))};
    case LUA_TSTRING: return Value{std::string(lua_tostring(L_, index))};
    default: return Value{};
    }
}

void LuaPart::push_params(const std::map<std::string, std::string> &params) {
    lua_newtable(L_);
    for (const auto &entry : params) {
        const std::string &text = entry.second;
        char *end = nullptr;
        const long long integer = std::strtoll(text.c_str(), &end, 0);
        if (!text.empty() && *end == '\0') {
            lua_pushinteger(L_, static_cast<lua_Integer>(integer));
        } else {
            end = nullptr;
            const double number = std::strtod(text.c_str(), &end);
            if (!text.empty() && *end == '\0' && std::isfinite(number))
                lua_pushnumber(L_, number);
            else if (text == "true" || text == "false")
                lua_pushboolean(L_, text == "true");
            else
                lua_pushlstring(L_, text.data(), text.size());
        }
        lua_setfield(L_, -2, entry.first.c_str());
    }
}

// ---- Lua-facing API ---------------------------------------------------

int LuaPart::l_now(lua_State *L) {
    lua_pushinteger(L, static_cast<lua_Integer>(self(L)->current_ns_));
    return 1;
}

int LuaPart::l_at(lua_State *L) {
    LuaPart *part = self(L);
    const lua_Integer when = luaL_checkinteger(L, 1);
    luaL_checktype(L, 2, LUA_TFUNCTION);
    if (when < 0)
        return luaL_error(L, "at(): time must not be negative");
    lua_pushvalue(L, 2);
    const int ref = luaL_ref(L, LUA_REGISTRYINDEX);
    const std::uint64_t id = part->next_timer_++;
    const EventId event = part->scheduler_.at(static_cast<std::uint64_t>(when),
                                              [part, id] { part->fire_timer(id); });
    part->timers_[id] = {event, ref};
    lua_pushinteger(L, static_cast<lua_Integer>(id));
    return 1;
}

int LuaPart::l_after(lua_State *L) {
    LuaPart *part = self(L);
    const lua_Integer delay = luaL_checkinteger(L, 1);
    if (delay < 0)
        return luaL_error(L, "after(): delay must not be negative");
    lua_pushinteger(L, static_cast<lua_Integer>(part->current_ns_) + delay);
    lua_replace(L, 1);
    return l_at(L);
}

int LuaPart::l_cancel(lua_State *L) {
    LuaPart *part = self(L);
    const auto id = static_cast<std::uint64_t>(luaL_checkinteger(L, 1));
    const auto it = part->timers_.find(id);
    if (it == part->timers_.end()) {
        lua_pushboolean(L, 0);
        return 1;
    }
    part->scheduler_.cancel(it->second.event);
    luaL_unref(L, LUA_REGISTRYINDEX, it->second.ref);
    part->timers_.erase(it);
    lua_pushboolean(L, 1);
    return 1;
}

void LuaPart::fire_timer(std::uint64_t id) {
    const auto it = timers_.find(id);
    if (it == timers_.end())
        return;
    const int ref = it->second.ref;
    timers_.erase(it);
    lua_rawgeti(L_, LUA_REGISTRYINDEX, ref);
    luaL_unref(L_, LUA_REGISTRYINDEX, ref);
    current_ns_ = scheduler_.now();
    lua_pushinteger(L_, static_cast<lua_Integer>(current_ns_));
    call(1, 0);
}

int LuaPart::l_expose(lua_State *L) {
    LuaPart *part = self(L);
    const std::string name = luaL_checkstring(L, 1);
    luaL_checktype(L, 2, LUA_TFUNCTION);
    const bool has_setter = !lua_isnoneornil(L, 3);
    if (has_setter)
        luaL_checktype(L, 3, LUA_TFUNCTION);
    if (name.empty() ||
        name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
                               "0123456789_") != std::string::npos)
        return luaL_error(L, "expose(): '%s' is not a valid name", name.c_str());
    for (const Exposed &e : part->exposed_)
        if (e.name == name)
            return luaL_error(L, "expose(): '%s' is already exposed", name.c_str());
    lua_pushvalue(L, 2);
    const int getter = luaL_ref(L, LUA_REGISTRYINDEX);
    int setter = LUA_NOREF;
    if (has_setter) {
        lua_pushvalue(L, 3);
        setter = luaL_ref(L, LUA_REGISTRYINDEX);
    }
    part->exposed_.push_back({name, getter, setter});
    return 0;
}

int LuaPart::l_rand(lua_State *L) {
    LuaPart *part = self(L);
    std::uint64_t x = part->rng_;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    part->rng_ = x;
    const std::uint64_t r = x * 2685821657736338717ull;
    lua_pushnumber(L, static_cast<double>(r >> 11) / 9007199254740992.0);
    return 1;
}

int LuaPart::l_log(lua_State *L) {
    LuaPart *part = self(L);
    std::ostringstream out;
    const int n = lua_gettop(L);
    for (int i = 1; i <= n; ++i) {
        out << (i > 1 ? "\t" : "") << luaL_tolstring(L, i, nullptr);
        lua_pop(L, 1);
    }
    std::cerr << "[" << part->name() << " @ " << part->current_ns_ << " ns] "
              << out.str() << '\n';
    return 0;
}

int LuaPart::pin_drive(lua_State *L) {
    PinSlot &slot = slot_of(L, 1);
    auto *handle = static_cast<PinHandle *>(lua_touserdata(L, 1));
    LuaPart *part = handle->part;
    const std::string state = luaL_checkstring(L, 2);
    Drive drive;
    if (state == "low") drive = Drive::Low;
    else if (state == "high") drive = Drive::High;
    else if (state == "hiz") drive = Drive::HiZ;
    else if (state == "pullup") drive = Drive::PullUp;
    else if (state == "pulldown") drive = Drive::PullDown;
    else return luaL_error(L, "drive(): unknown state '%s' (low, high, hiz, pullup, pulldown)",
                           state.c_str());
    const std::uint64_t when = lua_isnoneornil(L, 3)
                                   ? part->current_ns_
                                   : static_cast<std::uint64_t>(luaL_checkinteger(L, 3));
    // Edges caused by our own drive are not delivered to on_edge().
    part->in_own_drive_ = true;
    slot.pin->set_drive(drive, when);
    part->in_own_drive_ = false;
    return 0;
}

int LuaPart::pin_release(lua_State *L) {
    lua_settop(L, 1);
    lua_pushstring(L, "hiz");
    return pin_drive(L);
}

int LuaPart::pin_level(lua_State *L) {
    switch (slot_of(L, 1).pin->net().level()) {
    case Level::High: lua_pushboolean(L, 1); break;
    case Level::Low: lua_pushboolean(L, 0); break;
    default: lua_pushnil(L);
    }
    return 1;
}

int LuaPart::pin_level_at(lua_State *L) {
    PinSlot &slot = slot_of(L, 1);
    const lua_Integer when = luaL_checkinteger(L, 2);
    bool level = true;
    for (const auto &edge : slot.edges) {
        if (static_cast<lua_Integer>(edge.first) > when)
            break;
        level = edge.second;
    }
    lua_pushboolean(L, level);
    return 1;
}

int LuaPart::pin_mv(lua_State *L) {
    lua_pushinteger(L, static_cast<lua_Integer>(slot_of(L, 1).pin->net().millivolts()));
    return 1;
}

int LuaPart::pin_set_mv(lua_State *L) {
    PinSlot &slot = slot_of(L, 1);
    auto *handle = static_cast<PinHandle *>(lua_touserdata(L, 1));
    std::optional<std::uint32_t> mv;
    if (!lua_isnoneornil(L, 2)) {
        const lua_Integer value = luaL_checkinteger(L, 2);
        if (value < 0)
            return luaL_error(L, "set_mv(): voltage must not be negative");
        mv = static_cast<std::uint32_t>(value);
    }
    handle->part->in_own_drive_ = true;
    slot.pin->set_analog_mv(mv, handle->part->current_ns_);
    handle->part->in_own_drive_ = false;
    return 0;
}

int LuaPart::pin_name(lua_State *L) {
    lua_pushstring(L, slot_of(L, 1).name.c_str());
    return 1;
}

namespace {

bool parse_protocol(const std::string &name, tc32emu::RadioProtocol &out) {
    if (name == "zigbee") {
        out = tc32emu::RadioProtocol::Zigbee802154;
        return true;
    }
    if (name == "ble") {
        out = tc32emu::RadioProtocol::Ble;
        return true;
    }
    return false;
}

} // namespace

int LuaPart::radio_transmit(lua_State *L) {
    LuaPart *part = self(L);
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getfield(L, 1, "protocol");
    const std::string protocol_name = luaL_checkstring(L, -1);
    lua_pop(L, 1);
    tc32emu::RadioProtocol protocol;
    if (!parse_protocol(protocol_name, protocol))
        return luaL_error(L, "radio.transmit(): protocol must be \"zigbee\" or \"ble\"");
    bool declared = false;
    for (const std::string &name : part->radio_protocols_)
        declared |= name == protocol_name;
    if (!declared)
        return luaL_error(L, "radio.transmit(): the part's radio does not declare '%s'",
                          protocol_name.c_str());
    lua_getfield(L, 1, "channel");
    const lua_Integer channel = luaL_checkinteger(L, -1);
    lua_pop(L, 1);

    std::vector<std::uint8_t> payload;
    lua_getfield(L, 1, "payload");
    if (lua_type(L, -1) == LUA_TSTRING) {
        std::size_t length = 0;
        const char *bytes = lua_tolstring(L, -1, &length);
        payload.assign(bytes, bytes + length);
    } else if (lua_type(L, -1) == LUA_TTABLE) {
        const lua_Integer count = luaL_len(L, -1);
        for (lua_Integer i = 1; i <= count; ++i) {
            lua_geti(L, -1, i);
            const lua_Integer value = luaL_checkinteger(L, -1);
            if (value < 0 || value > 255)
                return luaL_error(L, "radio.transmit(): payload bytes must be 0..255");
            payload.push_back(static_cast<std::uint8_t>(value));
            lua_pop(L, 1);
        }
    } else {
        return luaL_error(L, "radio.transmit(): payload must be a string or a list of bytes");
    }
    lua_pop(L, 1);
    lua_getfield(L, 1, "to");
    const std::string destination = lua_isnil(L, -1) ? "" : luaL_checkstring(L, -1);
    lua_pop(L, 1);
    lua_getfield(L, 1, "t");
    const std::uint64_t start = lua_isnil(L, -1)
                                    ? part->current_ns_
                                    : static_cast<std::uint64_t>(luaL_checkinteger(L, -1));
    lua_pop(L, 1);

    std::uint64_t id = 0;
    try {
        id = part->air_->transmit(part->radio_node_, destination, protocol,
                                  static_cast<std::uint8_t>(channel), start,
                                  std::move(payload));
    } catch (const std::exception &error) {
        return luaL_error(L, "radio.transmit(): %s", error.what());
    }
    lua_pushinteger(L, static_cast<lua_Integer>(id));
    lua_pushinteger(L, static_cast<lua_Integer>(part->air_->frames().back().end_ns));
    return 2;
}

int LuaPart::radio_listen(lua_State *L) {
    LuaPart *part = self(L);
    if (lua_isnoneornil(L, 1)) {
        part->listening_ = false;
        return 0;
    }
    const std::string name = luaL_checkstring(L, 1);
    tc32emu::RadioProtocol protocol;
    if (!parse_protocol(name, protocol))
        return luaL_error(L, "radio.listen(): protocol must be \"zigbee\" or \"ble\"");
    bool declared = false;
    for (const std::string &declared_name : part->radio_protocols_)
        declared |= declared_name == name;
    if (!declared)
        return luaL_error(L, "radio.listen(): the part's radio does not declare '%s'",
                          name.c_str());
    const lua_Integer channel = luaL_checkinteger(L, 2);
    if (channel < 0 || channel > 255)
        return luaL_error(L, "radio.listen(): invalid channel");
    part->listening_ = true;
    part->listen_protocol_ = protocol;
    part->listen_channel_ = static_cast<std::uint8_t>(channel);
    return 0;
}

int LuaPart::radio_set_power(lua_State *L) {
    LuaPart *part = self(L);
    tc32emu::RadioNodeConfig node =
        part->air_->node(part->radio_node_) ? *part->air_->node(part->radio_node_)
                                            : tc32emu::RadioNodeConfig{};
    node.tx_power_dbm = luaL_checknumber(L, 1);
    part->air_->set_node(part->radio_node_, node);
    return 0;
}

int LuaPart::radio_airtime(lua_State *L) {
    tc32emu::RadioProtocol protocol;
    if (!parse_protocol(luaL_checkstring(L, 1), protocol))
        return luaL_error(L, "radio.airtime(): protocol must be \"zigbee\" or \"ble\"");
    const lua_Integer bytes = luaL_checkinteger(L, 2);
    if (bytes < 0)
        return luaL_error(L, "radio.airtime(): length must not be negative");
    lua_pushinteger(L, static_cast<lua_Integer>(tc32emu::VirtualAir::airtime_ns(
                           protocol, static_cast<std::size_t>(bytes))));
    return 1;
}

void LuaPart::on_air_frame(const tc32emu::RadioFrame &frame) {
    if (!listening_ || failed_ || frame.source == radio_node_ ||
        frame.protocol != listen_protocol_ || frame.channel != listen_channel_)
        return;
    // Decide at the end of the frame, when every frame that overlaps it has
    // been registered (all MCUs have advanced past that time by then).
    const auto protocol = frame.protocol;
    const auto channel = frame.channel;
    const std::uint64_t end = frame.end_ns;
    scheduler_.at(end, [this, protocol, channel, end] {
        deliver_frame(protocol, channel, end);
    });
}

void LuaPart::deliver_frame(tc32emu::RadioProtocol protocol, std::uint8_t channel,
                            std::uint64_t end_ns) {
    if (!listening_ || listen_protocol_ != protocol || listen_channel_ != channel)
        return;
    for (const tc32emu::RadioFrame &frame :
         air_->receive(radio_node_, protocol, channel, end_ns, end_ns - 1)) {
        lua_getglobal(L_, "on_frame");
        if (!lua_isfunction(L_, -1)) {
            lua_pop(L_, 1);
            return;
        }
        current_ns_ = end_ns;
        lua_newtable(L_);
        lua_pushinteger(L_, static_cast<lua_Integer>(frame.id));
        lua_setfield(L_, -2, "id");
        lua_pushstring(L_, protocol == tc32emu::RadioProtocol::Zigbee802154 ? "zigbee" : "ble");
        lua_setfield(L_, -2, "protocol");
        lua_pushinteger(L_, frame.channel);
        lua_setfield(L_, -2, "channel");
        lua_pushstring(L_, frame.source.c_str());
        lua_setfield(L_, -2, "source");
        if (!frame.destination.empty()) {
            lua_pushstring(L_, frame.destination.c_str());
            lua_setfield(L_, -2, "destination");
        }
        lua_pushlstring(L_, reinterpret_cast<const char *>(frame.payload.data()),
                        frame.payload.size());
        lua_setfield(L_, -2, "payload");
        lua_pushinteger(L_, frame.rssi_dbm);
        lua_setfield(L_, -2, "rssi_dbm");
        lua_pushinteger(L_, static_cast<lua_Integer>(frame.start_ns));
        lua_setfield(L_, -2, "start_ns");
        lua_pushinteger(L_, static_cast<lua_Integer>(frame.end_ns));
        lua_setfield(L_, -2, "end_ns");
        call(1, 0);
    }
}

int LuaPart::ref_function_field(int table_index, const char *field) {
    lua_getfield(L_, table_index, field);
    if (lua_isnil(L_, -1)) {
        lua_pop(L_, 1);
        return LUA_NOREF;
    }
    if (!lua_isfunction(L_, -1))
        luaL_error(L_, "i2c_slave(): '%s' must be a function", field);
    const int ref = luaL_ref(L_, LUA_REGISTRYINDEX);
    owned_refs_.push_back(ref);
    return ref;
}

int LuaPart::l_i2c_slave(lua_State *L) {
    LuaPart *part = self(L);
    luaL_checktype(L, 1, LUA_TTABLE);
    auto terminal = [&](const char *field, const char *fallback) {
        lua_getfield(L, 1, field);
        const std::string name = lua_isnil(L, -1) ? fallback : luaL_checkstring(L, -1);
        lua_pop(L, 1);
        for (std::size_t i = 0; i < part->pins_.size(); ++i)
            if (part->pins_[i].name == name)
                return i;
        luaL_error(L, "i2c_slave(): terminal '%s' is not wired", name.c_str());
        return std::size_t{0};
    };
    const std::size_t sda = terminal("sda", "sda");
    const std::size_t scl = terminal("scl", "scl");

    lua_getfield(L, 1, "address");
    const bool has_address = !lua_isnil(L, -1);
    const lua_Integer address = has_address ? luaL_checkinteger(L, -1) : -1;
    lua_pop(L, 1);
    const int on_address = part->ref_function_field(1, "on_address");
    const int on_write = part->ref_function_field(1, "on_write");
    const int on_read = part->ref_function_field(1, "on_read");
    const int on_start = part->ref_function_field(1, "on_start");
    const int on_stop = part->ref_function_field(1, "on_stop");
    if (!has_address && on_address == LUA_NOREF)
        return luaL_error(L, "i2c_slave(): 'address' or 'on_address' is required");
    if (has_address && (address < 0 || address > 0x7f))
        return luaL_error(L, "i2c_slave(): address must be a 7-bit value");

    // Handlers run from the net's change notification, which carries no
    // timestamp; now() inside them is the scheduler time.
    auto prepare = [part](int ref) {
        lua_rawgeti(part->L_, LUA_REGISTRYINDEX, ref);
        part->current_ns_ = std::max(part->current_ns_, part->scheduler_.now());
    };
    I2cSlave::Handlers handlers;
    handlers.on_address = [=](std::uint8_t a, bool read) {
        if (on_address == LUA_NOREF)
            return a == address;
        prepare(on_address);
        lua_pushinteger(part->L_, a);
        lua_pushboolean(part->L_, read);
        bool ack = false;
        if (part->call(2, 1)) {
            ack = lua_toboolean(part->L_, -1) != 0;
            lua_pop(part->L_, 1);
        }
        return ack;
    };
    if (on_write != LUA_NOREF)
        handlers.on_write = [=](std::uint8_t byte) {
            prepare(on_write);
            lua_pushinteger(part->L_, byte);
            bool ack = false;
            if (part->call(1, 1)) {
                ack = !lua_isboolean(part->L_, -1) || lua_toboolean(part->L_, -1);
                lua_pop(part->L_, 1);
            }
            return ack;
        };
    else
        handlers.on_write = [](std::uint8_t) { return false; };
    if (on_read != LUA_NOREF)
        handlers.on_read = [=]() -> std::uint8_t {
            prepare(on_read);
            std::uint8_t value = 0xff;
            if (part->call(0, 1)) {
                if (lua_isinteger(part->L_, -1))
                    value = static_cast<std::uint8_t>(lua_tointeger(part->L_, -1));
                lua_pop(part->L_, 1);
            }
            return value;
        };
    if (on_start != LUA_NOREF)
        handlers.on_start = [=] {
            prepare(on_start);
            part->call(0, 0);
        };
    if (on_stop != LUA_NOREF)
        handlers.on_stop = [=] {
            prepare(on_stop);
            part->call(0, 0);
        };
    part->i2c_slaves_.push_back(std::make_unique<I2cSlave>(
        *part->pins_[sda].pin, *part->pins_[scl].pin, std::move(handlers)));
    return 0;
}

// ---- construction ------------------------------------------------------

LuaPart::LuaPart(PartContext &ctx)
    : Part(ctx.name()), scheduler_(ctx.scheduler()) {
    std::uint64_t hash = 1469598103934665603ull ^ ctx.seed();
    for (char c : ctx.name())
        hash = (hash ^ static_cast<std::uint8_t>(c)) * 1099511628211ull;
    rng_ = hash ? hash : 0x9e3779b97f4a7c15ull;

    const std::string &script = ctx.script();
    if (script.size() < 4 || script.compare(script.size() - 4, 4, ".lua") != 0)
        throw std::invalid_argument("script '" + script +
                                    "' must be a .lua file (other languages are not supported yet)");

    // Pins first: the Lua state captures their addresses.
    pins_.reserve(ctx.terminals().size());
    for (const std::string &terminal : ctx.terminals()) {
        Pin &pin = ctx.pin(terminal);
        pins_.push_back({terminal, &pin, {}});
        pins_.back().edges.emplace_back(0, pin.net().level() != Level::Low);
        const std::size_t index = pins_.size() - 1;
        pin.net().subscribe(
            [this, index](const NetChange &change) { on_net_change(index, change); });
    }
    // Scripts accept any params; they are handed over as the `params` table.
    for (const auto &entry : ctx.raw_params())
        (void)ctx.params().string(entry.first);

    L_ = lua_newstate(limited_alloc, &allocator_);
    if (!L_)
        throw std::runtime_error("cannot create a Lua state");
    *static_cast<LuaPart **>(lua_getextraspace(L_)) = this;
    // Sandbox: no io/os/package/debug, no loading files, no wall-clock or
    // unseeded randomness. The globals below are the whole interface.
    static const luaL_Reg libs[] = {
        {LUA_GNAME, luaopen_base},     {LUA_TABLIBNAME, luaopen_table},
        {LUA_STRLIBNAME, luaopen_string}, {LUA_MATHLIBNAME, luaopen_math},
        {LUA_UTF8LIBNAME, luaopen_utf8},  {LUA_COLIBNAME, luaopen_coroutine},
    };
    for (const luaL_Reg &lib : libs) {
        luaL_requiref(L_, lib.name, lib.func, 1);
        lua_pop(L_, 1);
    }
    for (const char *name : {"dofile", "loadfile", "print", "collectgarbage"}) {
        lua_pushnil(L_);
        lua_setglobal(L_, name);
    }
    lua_getglobal(L_, "math");
    for (const char *name : {"random", "randomseed"}) {
        lua_pushnil(L_);
        lua_setfield(L_, -2, name);
    }
    lua_pop(L_, 1);

    const struct {
        const char *name;
        lua_CFunction fn;
    } functions[] = {
        {"now", l_now},     {"at", l_at},         {"after", l_after},
        {"cancel", l_cancel}, {"expose", l_expose}, {"i2c_slave", l_i2c_slave},
        {"rand", l_rand},   {"log", l_log},
    };
    for (const auto &fn : functions) {
        lua_pushcfunction(L_, fn.fn);
        lua_setglobal(L_, fn.name);
    }
    lua_pushinteger(L_, 1000);
    lua_setglobal(L_, "US");
    lua_pushinteger(L_, 1000000);
    lua_setglobal(L_, "MS");
    lua_pushinteger(L_, 1000000000);
    lua_setglobal(L_, "S");

    if (luaL_newmetatable(L_, PinMeta)) {
        lua_newtable(L_);
        const luaL_Reg methods[] = {
            {"drive", pin_drive},   {"release", pin_release}, {"level", pin_level},
            {"level_at", pin_level_at}, {"mv", pin_mv},       {"set_mv", pin_set_mv},
            {"name", pin_name},     {nullptr, nullptr},
        };
        luaL_setfuncs(L_, methods, 0);
        lua_setfield(L_, -2, "__index");
    }
    lua_pop(L_, 1);

    lua_newtable(L_); // pins
    for (std::size_t i = 0; i < pins_.size(); ++i) {
        auto *handle = static_cast<PinHandle *>(lua_newuserdatauv(L_, sizeof(PinHandle), 0));
        *handle = {this, i};
        luaL_setmetatable(L_, PinMeta);
        lua_setfield(L_, -2, pins_[i].name.c_str());
    }
    lua_setglobal(L_, "pins");
    push_params(ctx.raw_params());
    lua_setglobal(L_, "params");

    if (ctx.air()) {
        air_ = ctx.air();
        radio_node_ = ctx.radio_node();
        radio_protocols_ = ctx.radio_protocols();
        lua_newtable(L_);
        const luaL_Reg radio_functions[] = {
            {"transmit", radio_transmit}, {"listen", radio_listen},
            {"set_power", radio_set_power}, {"airtime", radio_airtime},
            {nullptr, nullptr},
        };
        luaL_setfuncs(L_, radio_functions, 0);
        lua_setglobal(L_, "radio");
        air_->add_observer(
            [this](const tc32emu::RadioFrame &frame) { on_air_frame(frame); });
    }

    lua_sethook(L_, hook, LUA_MASKCOUNT, HookInterval);
    current_ns_ = scheduler_.now();
    if (luaL_loadfile(L_, script.c_str()) != LUA_OK) {
        const std::string message = lua_tostring(L_, -1);
        throw std::runtime_error(message);
    }
    hook_count_ = 0;
    lua_pushcfunction(L_, traceback);
    lua_insert(L_, -2);
    if (lua_pcall(L_, 0, 0, -2) != LUA_OK) {
        const char *message = lua_tostring(L_, -1);
        throw std::runtime_error(message ? message : "script failed to load");
    }
    lua_pop(L_, 1);
}

LuaPart::~LuaPart() {
    if (L_)
        lua_close(L_);
}

void LuaPart::on_net_change(std::size_t index, const NetChange &change) {
    PinSlot &slot = pins_[index];
    slot.edges.emplace_back(change.time_ns, change.level != Level::Low);
    while (slot.edges.size() > EdgeHistory)
        slot.edges.pop_front();
    if (in_own_drive_ || failed_)
        return;
    lua_getglobal(L_, "on_edge");
    if (!lua_isfunction(L_, -1)) {
        lua_pop(L_, 1);
        return;
    }
    current_ns_ = change.time_ns;
    lua_pushstring(L_, slot.name.c_str());
    lua_pushboolean(L_, change.level != Level::Low);
    lua_pushinteger(L_, static_cast<lua_Integer>(change.time_ns));
    call(3, 0);
}

void LuaPart::register_endpoints(EndpointRegistry &registry,
                                 const std::function<std::uint64_t()> &now) {
    for (const Exposed &item : exposed_) {
        const std::string full = name() + "." + item.name;
        const int getter = item.getter_ref;
        const int setter = item.setter_ref;
        registry.add(
            full,
            std::make_unique<ParamEndpoint>(
                full,
                [this, getter, now]() -> Value {
                    lua_rawgeti(L_, LUA_REGISTRYINDEX, getter);
                    current_ns_ = now();
                    Value value;
                    if (call(0, 1, true)) {
                        value = to_value(-1);
                        lua_pop(L_, 1);
                    }
                    return value;
                },
                [this, setter, now](const Value &value) {
                    if (setter == LUA_NOREF)
                        throw std::logic_error("parameter is read-only");
                    lua_rawgeti(L_, LUA_REGISTRYINDEX, setter);
                    current_ns_ = now();
                    push_value(value);
                    call(1, 0, true);
                },
                now));
    }
}

std::unique_ptr<Part> make_lua_part(PartContext &context) {
    return std::make_unique<LuaPart>(context);
}

} // namespace sim
