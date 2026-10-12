#include "sim/cli.hpp"

#include "sim/config.hpp"
#include "sim/system.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace sim {

void print_sim_usage(std::ostream &out) {
    out << "       tc32-emu sim SYSTEM.yaml [--for DURATION] [--quantum DURATION]\n"
           "       [--trace-nets] [--attach ENDPOINT=SINK]... [--list-endpoints]\n"
           "       sinks: stdout, file:PATH, tcp:PORT, vcd:PATH, pcap:PATH[,ble]\n";
}

namespace {

const char *state_name(McuState state) {
    switch (state) {
    case McuState::Running: return "running";
    case McuState::Sleeping: return "sleeping";
    case McuState::Halted: return "halted";
    case McuState::Fault: return "fault";
    }
    return "?";
}

} // namespace

int sim_command(int argc, char **argv) {
    std::string path;
    std::optional<std::uint64_t> duration, quantum;
    bool trace_nets = false;
    bool list_endpoints = false;
    std::vector<std::pair<std::string, std::string>> attachments;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--for" && i + 1 < argc)
            duration = parse_duration_ns(argv[++i]);
        else if (arg == "--quantum" && i + 1 < argc)
            quantum = parse_duration_ns(argv[++i]);
        else if (arg == "--trace-nets")
            trace_nets = true;
        else if (arg == "--list-endpoints")
            list_endpoints = true;
        else if (arg == "--attach" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const std::size_t equals = spec.find('=');
            if (equals == std::string::npos || equals == 0 || equals + 1 == spec.size())
                throw std::invalid_argument("--attach must be ENDPOINT=SINK: " + spec);
            attachments.emplace_back(spec.substr(0, equals), spec.substr(equals + 1));
        }
        else if (arg == "--help" || arg == "-h") {
            print_sim_usage(std::cout);
            return 0;
        } else if (!arg.empty() && arg[0] != '-' && path.empty())
            path = arg;
        else
            throw std::invalid_argument("unknown or incomplete option: " + arg);
    }
    if (path.empty())
        throw std::invalid_argument("sim requires a SYSTEM.yaml path");

    SystemConfig config = load_system_config(path);
    if (quantum)
        config.quantum_ns = quantum;
    const auto system = build_system(config);
    World &world = system->world();
    for (const std::string &warning : system->warnings())
        std::cerr << "warning: " << warning << '\n';

    if (list_endpoints) {
        for (const std::string &name : world.endpoints().names())
            std::cout << name << "  (" << world.endpoints().at(name).kind() << ")\n";
        return 0;
    }
    for (const auto &[endpoint, sink] : attachments) {
        try {
            system->session().attach(endpoint, sink);
        } catch (const std::out_of_range &) {
            throw std::invalid_argument("--attach: unknown endpoint '" + endpoint +
                                        "' (see --list-endpoints)");
        }
    }

    const std::optional<std::uint64_t> run_for =
        duration ? duration : system->duration_ns();
    if (!run_for)
        throw std::invalid_argument(
            "no run length: pass --for DURATION or set world.duration");

    if (trace_nets) {
        for (const std::string &name : world.endpoints().names()) {
            Endpoint &endpoint = world.endpoints().at(name);
            if (endpoint.kind() != "net")
                continue;
            endpoint.subscribe([](const EndpointEvent &event) {
                std::cout << "net " << event.endpoint << " = "
                          << (std::get<bool>(event.value) ? 1 : 0) << " at "
                          << event.time_ns << " ns\n";
            });
        }
    }

    world.run_until(*run_for);
    system->session().finish();
    std::cout << "simulated " << world.now() << " ns\n";
    for (const std::string &name : world.mcu_names()) {
        Mcu &mcu = world.mcu(name);
        std::cout << "mcu " << name << " (" << mcu.model() << "): "
                  << state_name(mcu.state()) << " at " << mcu.time_ns() << " ns";
        if (!mcu.fault_message().empty())
            std::cout << " - " << mcu.fault_message();
        std::cout << '\n';
    }
    for (const std::string &name : world.endpoints().names()) {
        Endpoint &endpoint = world.endpoints().at(name);
        if (endpoint.kind() == "net")
            std::cout << "net " << name << " = "
                      << (std::get<bool>(endpoint.get()) ? 1 : 0) << '\n';
    }
    return 0;
}

} // namespace sim
