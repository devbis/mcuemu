#include "sim/cli.hpp"

#include "sim/config.hpp"
#include "sim/server.hpp"
#include "sim/system.hpp"

#include <csignal>
#include <iostream>
#include <string>
#include <vector>

namespace sim {

void print_sim_usage(std::ostream &out) {
    out << "       tc32-emu sim SYSTEM.yaml [--for DURATION] [--quantum DURATION]\n"
           "       [--trace-nets] [--attach ENDPOINT=SINK]... [--list-endpoints]\n"
           "       [--trace-uart PATTERN]...\n"
           "       sinks: stdout, hex[:GAP], file:PATH, tcp:PORT, vcd:PATH, pcap:PATH[,ble]\n"
           "       --trace-uart prints the bytes of every UART endpoint whose name contains\n"
           "       PATTERN (or all of them for '*') as hex lines, one per frame\n";
}

void print_serve_usage(std::ostream &out) {
    out << "       tc32-emu serve --root DIR [--port N] [--bind ADDR] [--token T]\n"
           "       [--allow-origin ORIGIN]... [--max-sessions N] [--idle-timeout SEC]\n";
}

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;
void request_stop(int) { g_stop_requested = 1; }

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

int serve_command(int argc, char **argv) {
    ServerOptions options;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            const unsigned long port = std::stoul(argv[++i]);
            if (port > 65535)
                throw std::invalid_argument("--port must be 0..65535");
            options.port = static_cast<unsigned short>(port);
        } else if (arg == "--bind" && i + 1 < argc) {
            options.bind = argv[++i];
        } else if (arg == "--root" && i + 1 < argc) {
            options.root = argv[++i];
        } else if (arg == "--token" && i + 1 < argc) {
            options.token = argv[++i];
        } else if (arg == "--allow-origin" && i + 1 < argc) {
            options.allowed_origins.push_back(argv[++i]);
        } else if (arg == "--max-sessions" && i + 1 < argc) {
            options.max_sessions = std::stoul(argv[++i]);
        } else if (arg == "--idle-timeout" && i + 1 < argc) {
            options.idle_timeout = std::chrono::seconds(std::stoul(argv[++i]));
        } else if (arg == "--help" || arg == "-h") {
            print_serve_usage(std::cout);
            return 0;
        } else {
            throw std::invalid_argument("unknown or incomplete option: " + arg);
        }
    }
    if (options.root.empty())
        throw std::invalid_argument("serve requires --root DIR (files are confined to it)");
    Server server(options);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    std::cout << "listening on ws://" << options.bind << ":" << server.port() << "/ (root "
              << options.root << ")" << std::endl;
    while (!g_stop_requested)
        server.run_once(100);
    std::cout << "stopped" << std::endl;
    return 0;
}

int sim_command(int argc, char **argv) {
    std::string path;
    std::optional<std::uint64_t> duration, quantum;
    bool trace_nets = false;
    bool list_endpoints = false;
    std::vector<std::string> uart_patterns;
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
        else if (arg == "--trace-uart" && i + 1 < argc)
            uart_patterns.push_back(argv[++i]);
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

    for (const std::string &pattern : uart_patterns) {
        bool matched = false;
        for (const std::string &name : world.endpoints().names()) {
            if (world.endpoints().at(name).kind() != "uart")
                continue;
            if (pattern != "*" && name.find(pattern) == std::string::npos)
                continue;
            system->session().attach(name, "hex");
            matched = true;
        }
        if (!matched)
            throw std::invalid_argument("--trace-uart: no UART endpoint matches '" + pattern +
                                        "' (see --list-endpoints)");
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
