#pragma once

#include "sim/mcu.hpp"
#include "sim/net.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace sim {

// Invalid or unsupported configuration. what() starts with "line N: " when the
// problem is attributable to a place in the YAML document.
class ConfigError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// A radio medium ("air"): the propagation model shared by the devices that
// attach to it.
struct RadioMediumConfig {
    std::string id;
    std::string model{"ideal"}; // ideal | log_distance
    double reference_loss_db{40.0};
    double exponent{2.0};
    double shadowing_sigma_db{0.0};
    double noise_floor_dbm{-100.0};
    double sinr_threshold_db{6.0};
    int line{0};
};

// How an MCU or part sits on a medium.
struct RadioAttachConfig {
    std::string air;
    std::array<double, 3> position{0, 0, 0}; // meters
    double tx_power_dbm{0.0};
    double sensitivity_dbm{-97.0};
    std::vector<std::string> protocols{"zigbee"}; // zigbee, ble
    int line{0};
};

// Connects an endpoint to a sink (see sim/sink.hpp for the spec grammar).
struct AttachConfig {
    std::string endpoint; // device scope: relative to the device; else full name
    std::string sink;
    int line{0};
};

// SEGGER RTT control block of the firmware, read from target memory.
struct RttConfig {
    std::uint32_t address{0};
    unsigned channels{1}; // endpoints "rtt", "rtt1", ... for channels 0..n-1
    std::uint64_t interval_ns{100000};
    int line{0};
};

struct McuConfig {
    std::string id;
    std::string model;
    std::optional<std::string> variant;
    std::string firmware; // resolved against the YAML file's directory
    // Every image to load; `firmware: path` gives one image at offset 0, a
    // list of {file, offset} places several (bootloader + application).
    std::vector<FirmwareImage> images;
    std::optional<std::uint32_t> clock_hz;
    std::optional<std::uint32_t> supply_mv; // supply voltage (default 3300)
    bool flash_timing{false}; // tlsr8258: charge flash fetches for cache misses
    std::uint64_t start_at_ns{0};
    std::optional<RadioAttachConfig> radio;
    std::optional<RttConfig> rtt;
    int line{0};
};

struct NetConfig {
    std::string id;
    NetKind kind{NetKind::PushPull};
    Drive pull{Drive::HiZ}; // HiZ, PullUp or PullDown
    // Pad references. Device scope: "mcu.PAD". World scope: "device.mcu.PAD".
    std::vector<std::string> pins;
    int line{0};
};

struct LinkConfig {
    std::string type; // "uart"
    // Endpoint references: "mcu.uartN" (device scope) or "device.mcu.uartN".
    std::string a;
    std::string b;
    std::uint32_t baud{115200};
    std::uint64_t latency_ns{0};
    int line{0};
};

// External peripheral (sensor, thermistor, ...). `pins` maps part terminals to
// nets of the same device (or full world net names); `params` are scalars
// interpreted by the part type.
struct PartConfig {
    std::string id;
    std::string type;
    std::map<std::string, std::string> pins;
    std::map<std::string, std::string> params;
    std::string script; // type "script": file resolved against the YAML dir
    std::optional<RadioAttachConfig> radio;
    int line{0};
};

struct DeviceConfig {
    std::string id;
    std::vector<McuConfig> mcus;
    std::vector<PartConfig> parts;
    std::vector<AttachConfig> attach;
    std::vector<NetConfig> nets;
    std::vector<LinkConfig> links;
    int line{0};
};

// External stimulus at a given time: either drive a net from outside the
// devices, or set an endpoint (for example a part parameter).
struct EventConfig {
    std::uint64_t at_ns{0};
    std::string net; // full net name ("device.net" or a world-level net)
    Drive drive{Drive::HiZ};
    std::string endpoint; // set when this event sets an endpoint instead
    std::string value;    // scalar text: true/false, integer, number or string
    int line{0};
};

struct SystemConfig {
    std::uint64_t seed{1};
    std::optional<std::uint64_t> duration_ns;
    std::optional<std::uint64_t> quantum_ns;
    std::vector<RadioMediumConfig> radios;
    std::vector<DeviceConfig> devices;
    std::vector<NetConfig> nets;   // world-level, full pad references
    std::vector<LinkConfig> links; // world-level, full endpoint references
    std::vector<AttachConfig> attach; // world-level, full endpoint names
    std::vector<EventConfig> events;
};

// "10ns", "5us", "100ms", "2.5s", "0".
std::uint64_t parse_duration_ns(const std::string &text);
// "48000000", "48MHz", "32kHz", "1Hz".
std::uint32_t parse_frequency_hz(const std::string &text);

// Parses and structurally validates a system description. Relative firmware
// paths are resolved against `base_dir`. Keys that belong to planned but not
// yet implemented features (parts, radio, attach, script) are rejected with a
// clear message rather than silently ignored.
//
// A `devices` entry may be `{include: file.yaml, id?, params?}`: the file holds
// one `device:` (optionally with default `params:`); `${name}` scalars inside
// it are replaced by the parameters. `reader` returns the text of an included
// file given its resolved path (default: read from disk); a hosting service
// uses it to confine the files that may be read.
using IncludeReader = std::function<std::string(const std::string &path)>;
SystemConfig parse_system_config(const std::string &yaml_text,
                                 const std::string &base_dir = ".",
                                 const IncludeReader &reader = {});
SystemConfig load_system_config(const std::string &path);

} // namespace sim
