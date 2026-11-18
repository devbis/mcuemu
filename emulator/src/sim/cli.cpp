#include "sim/cli.hpp"

#include "sim/config.hpp"
#include "sim/server.hpp"
#include "sim/system.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace sim {

void print_sim_usage(std::ostream &out) {
    out << "       mcuemu sim SYSTEM.yaml [--for DURATION] [--quantum DURATION] [--speed X]\n"
           "       [--trace-nets] [--attach ENDPOINT=SINK]... [--list-endpoints]\n"
           "       [--trace-uart PATTERN[=hex|text]]...\n"
           "       sinks: stdout, text, hex[:GAP], file:PATH, tcp:PORT, vcd:PATH, pcap:PATH[,ble]\n"
           "       --speed X paces virtual time at X times the wall clock (1 = real time) so\n"
           "       an outside program can talk to the system over a tcp: sink; Ctrl-C stops\n"
           "       --trace-uart prints the bytes of every UART endpoint whose name contains\n"
           "       PATTERN ('*' for all) as hex lines, one per frame (default), or as text lines\n";
}

void print_serve_usage(std::ostream &out) {
    out << "       mcuemu serve --root DIR [--port N] [--bind ADDR] [--token T]\n"
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
    double speed = 0;
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
        else if (arg == "--speed" && i + 1 < argc) {
            try {
                std::size_t used = 0;
                const std::string text = argv[++i];
                speed = std::stod(text, &used);
                if (used != text.size() || speed < 0)
                    throw std::invalid_argument("negative");
            } catch (const std::exception &) {
                throw std::invalid_argument("--speed must be a number >= 0 (1 = real time, 0 = as fast as possible)");
            }
        }
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

    for (std::string pattern : uart_patterns) {
        std::string sink = "hex";
        if (const std::size_t equals = pattern.rfind('=');
            equals != std::string::npos) {
            sink = pattern.substr(equals + 1);
            pattern.resize(equals);
            if (sink != "hex" && sink != "text")
                throw std::invalid_argument("--trace-uart: the format must be hex or text");
        }
        bool matched = false;
        for (const std::string &name : world.endpoints().names()) {
            if (world.endpoints().at(name).kind() != "uart")
                continue;
            if (pattern != "*" && name.find(pattern) == std::string::npos)
                continue;
            system->session().attach(name, sink);
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

    if (speed > 0) {
        std::signal(SIGINT, request_stop);
        std::signal(SIGTERM, request_stop);
        // Paced: virtual time follows the wall clock times `speed`, in 100 us
        // slices (sinks such as tcp: are polled between them), so an outside
        // program (a BLE host stack on an HCI socket) can talk to the system.
        const auto wall_start = std::chrono::steady_clock::now();
        const std::uint64_t virtual_start = world.now();
        while (world.now() < *run_for && !g_stop_requested) {
            world.run_until(std::min(*run_for, world.now() + 100000));
            std::this_thread::sleep_until(
                wall_start + std::chrono::nanoseconds(static_cast<std::int64_t>(
                                 static_cast<double>(world.now() - virtual_start) / speed)));
        }
    } else {
        world.run_until(*run_for);
    }
    system->session().finish();
    std::cout << "simulated " << world.now() << " ns\n";
    bool faulted = false;
    for (const std::string &name : world.mcu_names()) {
        Mcu &mcu = world.mcu(name);
        faulted = faulted || mcu.state() == McuState::Fault;
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
    // Like `run`: 1 when the firmware faulted, 2 for usage or setup errors.
    return faulted ? 1 : 0;
}

} // namespace sim
