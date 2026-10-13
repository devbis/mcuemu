#include "sim/simulation.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace sim {

namespace fs = std::filesystem;

namespace {

// True if `path` is `root` or lies under it (both already canonical).
bool within(const fs::path &root, const fs::path &path) {
    auto root_it = root.begin();
    auto path_it = path.begin();
    for (; root_it != root.end(); ++root_it, ++path_it)
        if (path_it == path.end() || *root_it != *path_it)
            return false;
    return true;
}

} // namespace

Simulation::Simulation(std::string root) {
    std::error_code error;
    const fs::path canonical = fs::weakly_canonical(root, error);
    if (error || !fs::is_directory(canonical))
        throw SimulationError("root '" + root + "' is not a directory");
    root_ = canonical.string();
}

Simulation::~Simulation() { unload(); }

std::string Simulation::confine_path(const std::string &path,
                                     const std::string &base) const {
    fs::path resolved(path);
    if (resolved.is_relative())
        resolved = fs::path(root_) / base / resolved;
    std::error_code error;
    // weakly_canonical follows symlinks in the existing part, so a link inside
    // the root that points outside is caught as well.
    const fs::path canonical = fs::weakly_canonical(resolved, error);
    if (error || !within(fs::path(root_), canonical))
        throw SimulationError("path '" + path + "' is outside the allowed directory");
    return canonical.string();
}

std::string Simulation::confine_sink(const std::string &spec) const {
    const std::size_t colon = spec.find(':');
    const std::string kind = spec.substr(0, colon);
    const std::string arg = colon == std::string::npos ? "" : spec.substr(colon + 1);
    if (kind == "stdout" || kind == "hex" || kind == "text")
        throw SimulationError("the " + kind + " sink is not available to remote clients");
    if (kind == "file" || kind == "vcd")
        return kind + ":" + confine_path(arg);
    if (kind == "pcap") {
        const std::size_t comma = arg.rfind(',');
        const std::string path = comma == std::string::npos ? arg : arg.substr(0, comma);
        const std::string option = comma == std::string::npos ? "" : arg.substr(comma);
        return "pcap:" + confine_path(path) + option;
    }
    if (kind == "tcp") {
        // Loopback only, and no privileged ports, so a client cannot expose
        // the host or squat on well-known services.
        std::string host = "127.0.0.1", port_text = arg;
        if (const std::size_t sep = arg.rfind(':'); sep != std::string::npos) {
            host = arg.substr(0, sep);
            port_text = arg.substr(sep + 1);
        }
        if (host != "127.0.0.1" && host != "localhost")
            throw SimulationError("tcp sinks may only listen on 127.0.0.1");
        const unsigned long port = port_text.find_first_not_of("0123456789") == std::string::npos &&
                                           !port_text.empty() && port_text.size() <= 5
                                       ? std::stoul(port_text)
                                       : 70000;
        if (port != 0 && port < 1024)
            throw SimulationError("tcp sinks may not use ports below 1024");
        return spec;
    }
    return spec; // unknown kinds are rejected by the session itself
}

void Simulation::check_config(const SystemConfig &config) const {
    auto check_attach = [&](const AttachConfig &attach) {
        (void)confine_sink(attach.sink);
    };
    for (const AttachConfig &attach : config.attach)
        check_attach(attach);
    for (const DeviceConfig &device : config.devices) {
        for (const AttachConfig &attach : device.attach)
            check_attach(attach);
        for (const McuConfig &mcu : device.mcus)
            for (const FirmwareImage &image : mcu.images)
                (void)confine_path(image.file);
        for (const PartConfig &part : device.parts)
            if (!part.script.empty())
                (void)confine_path(part.script);
    }
}

void Simulation::load(const std::string &yaml, const std::string &base_dir) {
    constexpr std::size_t MaxYaml = 1u << 20;
    if (yaml.size() > MaxYaml)
        throw SimulationError("the description is larger than 1 MiB");
    const std::string base = confine_path(base_dir.empty() ? "." : base_dir);
    // Included device files are confined like every other file.
    const IncludeReader reader = [this](const std::string &path) {
        const std::string confined = confine_path(path);
        std::ifstream input(confined, std::ios::binary);
        if (!input)
            throw SimulationError("cannot open included file '" + path + "'");
        std::ostringstream text;
        text << input.rdbuf();
        return text.str();
    };
    SystemConfig config = parse_system_config(yaml, base, reader);
    // Sink paths in the YAML are relative to the root, not to the base dir.
    for (AttachConfig &attach : config.attach)
        attach.sink = confine_sink(attach.sink);
    for (DeviceConfig &device : config.devices)
        for (AttachConfig &attach : device.attach)
            attach.sink = confine_sink(attach.sink);
    check_config(config);
    auto fresh = build_system(config);
    unload();
    system_ = std::move(fresh);
    running_ = false;
    target_ns_.reset();
    last_states_.clear();
    for (const std::string &name : system_->world().mcu_names())
        last_states_[name] = McuState::Running;
}

void Simulation::unload() {
    for (auto &entry : subscriptions_)
        if (system_)
            entry.second.endpoint->unsubscribe(entry.second.endpoint_subscription);
    subscriptions_.clear();
    events_.clear();
    system_.reset();
    running_ = false;
    target_ns_.reset();
}

System &Simulation::system() {
    if (!system_)
        throw SimulationError("no system is loaded");
    return *system_;
}

std::uint64_t Simulation::time_ns() { return system().world().now(); }

void Simulation::run(std::optional<std::uint64_t> target_ns) {
    system();
    target_ns_ = target_ns;
    running_ = !target_ns || *target_ns > time_ns();
    anchor();
}

void Simulation::anchor() {
    anchor_wall_ = std::chrono::steady_clock::now();
    anchor_virtual_ = system_ ? system_->world().now() : 0;
}

namespace {
// Paced runs execute in slices of at least this much virtual time.
constexpr std::uint64_t MinPacedSliceNs = 20000;
} // namespace

std::uint64_t Simulation::allowed_ns() {
    constexpr std::uint64_t MaxLagNs = 50000000; // 50 ms of virtual time
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now() - anchor_wall_).count();
    std::uint64_t allowed = anchor_virtual_ + static_cast<std::uint64_t>(elapsed * speed_);
    const std::uint64_t now = system_->world().now();
    if (allowed > now + MaxLagNs) { // fell behind: drop the backlog
        anchor_virtual_ = now;
        anchor_wall_ = std::chrono::steady_clock::now();
        allowed = now;
    }
    return allowed;
}

void Simulation::set_speed(double speed) {
    if (!(speed == 0 || (speed >= 0.001 && speed <= 1000)))
        throw SimulationError("the speed must be 0 (maximum) or between 0.001 and 1000");
    speed_ = speed;
    anchor();
}

bool Simulation::throttled() {
    return system_ && running_ && speed_ > 0 && allowed_ns() < system_->world().now() + MinPacedSliceNs;
}

void Simulation::pause() {
    running_ = false;
    target_ns_.reset();
}

void Simulation::set_slice_ns(std::uint64_t slice_ns) {
    if (slice_ns == 0)
        throw SimulationError("the slice must be positive");
    slice_ns_ = slice_ns;
}

std::vector<McuStop> Simulation::collect_stops() {
    std::vector<McuStop> stops;
    World &world = system_->world();
    for (const std::string &name : world.mcu_names()) {
        Mcu &mcu = world.mcu(name);
        McuState &last = last_states_[name];
        if (mcu.state() != last) {
            last = mcu.state();
            if (mcu.state() != McuState::Running)
                stops.push_back({name, mcu.state(), mcu.fault_message(), mcu.time_ns()});
        }
    }
    return stops;
}

Simulation::PumpResult Simulation::pump(std::chrono::milliseconds wall_budget) {
    PumpResult result;
    if (!system_ || !running_)
        return result;
    World &world = system_->world();
    const auto deadline = std::chrono::steady_clock::now() + wall_budget;
    do {
        std::uint64_t next = world.now() + slice_ns_;
        if (speed_ > 0) {
            const std::uint64_t allowed = allowed_ns();
            if (allowed < world.now() + MinPacedSliceNs)
                break; // not worth a slice yet; wait for wall time to pass
            next = std::min(next, allowed);
        }
        if (target_ns_)
            next = std::min(next, *target_ns_);
        world.run_until(next);
        auto stops = collect_stops();
        result.stops.insert(result.stops.end(), stops.begin(), stops.end());
        if (target_ns_ && world.now() >= *target_ns_) {
            running_ = false;
            target_ns_.reset();
            result.reached_target = true;
            break;
        }
    } while (running_ && std::chrono::steady_clock::now() < deadline);
    return result;
}

std::size_t Simulation::subscribe(const std::string &endpoint_name) {
    Endpoint *endpoint = system().world().endpoints().find(endpoint_name);
    if (!endpoint)
        throw SimulationError("unknown endpoint '" + endpoint_name + "'");
    const std::size_t id = next_subscription_++;
    const std::size_t inner = endpoint->subscribe([this, id](const EndpointEvent &event) {
        constexpr std::size_t MaxQueued = 1u << 20; // events, not bytes
        if (events_.size() < MaxQueued)
            events_.push_back({id, event});
    });
    subscriptions_[id] = {endpoint, inner};
    return id;
}

void Simulation::unsubscribe(std::size_t subscription) {
    const auto it = subscriptions_.find(subscription);
    if (it == subscriptions_.end())
        throw SimulationError("unknown subscription " + std::to_string(subscription));
    it->second.endpoint->unsubscribe(it->second.endpoint_subscription);
    subscriptions_.erase(it);
    events_.erase(std::remove_if(events_.begin(), events_.end(),
                                 [&](const SimEvent &e) { return e.subscription == subscription; }),
                  events_.end());
}

std::vector<SimEvent> Simulation::take_events() {
    std::vector<SimEvent> taken(events_.begin(), events_.end());
    events_.clear();
    return taken;
}

std::size_t Simulation::attach(const std::string &endpoint, const std::string &spec) {
    try {
        return system().session().attach(endpoint, confine_sink(spec));
    } catch (const std::out_of_range &) {
        throw SimulationError("unknown endpoint '" + endpoint + "'");
    } catch (const SimulationError &) {
        throw;
    } catch (const std::exception &error) {
        throw SimulationError(error.what());
    }
}

void Simulation::detach(std::size_t id) {
    if (!system().session().detach(id))
        throw SimulationError("unknown attachment " + std::to_string(id));
}

Bytes Simulation::read_memory(const std::string &mcu_name, std::uint32_t address,
                              std::size_t length) {
    constexpr std::size_t MaxRead = 1u << 20;
    if (length > MaxRead)
        throw SimulationError("a single read is limited to 1 MiB");
    Mcu *mcu = nullptr;
    try {
        mcu = &system().world().mcu(mcu_name);
    } catch (const std::out_of_range &) {
        throw SimulationError("unknown MCU '" + mcu_name + "'");
    }
    Bytes data(length);
    try {
        mcu->read_memory(address, data.data(), length);
    } catch (const std::exception &error) {
        throw SimulationError(error.what());
    }
    return data;
}

void Simulation::write_memory(const std::string &mcu_name, std::uint32_t address,
                              const Bytes &data) {
    Mcu *mcu = nullptr;
    try {
        mcu = &system().world().mcu(mcu_name);
    } catch (const std::out_of_range &) {
        throw SimulationError("unknown MCU '" + mcu_name + "'");
    }
    try {
        mcu->write_memory(address, data.data(), data.size());
    } catch (const std::exception &error) {
        throw SimulationError(error.what());
    }
}

} // namespace sim
