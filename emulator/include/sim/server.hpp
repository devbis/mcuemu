#pragma once

#include "sim/rpc.hpp"
#include "sim/websocket.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace sim {

struct ServerOptions {
    std::string bind{"127.0.0.1"};
    unsigned short port{0}; // 0 picks a free port; see Server::port()
    // Every file a client can make the server read or write lives under here.
    std::string root;
    // If set, clients must present it as `?token=...` or `Authorization:
    // Bearer ...`. Required when binding to anything but loopback.
    std::string token;
    // Browser pages are refused unless their Origin is listed ("*" allows all);
    // clients that send no Origin header (scripts, tools) are not affected.
    std::vector<std::string> allowed_origins;
    std::size_t max_sessions{8};
    std::chrono::seconds idle_timeout{600};
    std::chrono::milliseconds pump_budget{5};
};

// WebSocket server for the JSON-RPC control API. One connection is one
// isolated simulation. Single-threaded: run_once() handles I/O and advances
// running simulations in short bursts.
class Server {
public:
    explicit Server(ServerOptions options);
    ~Server();
    Server(const Server &) = delete;
    Server &operator=(const Server &) = delete;

    unsigned short port() const noexcept { return port_; }
    std::size_t connections() const noexcept { return clients_.size(); }

    // One iteration: wait up to `timeout_ms` for I/O (not at all while a
    // simulation is running), serve it and advance running simulations.
    void run_once(int timeout_ms);
    void run();
    void stop() { stopping_ = true; }

private:
    struct Client;
    void accept_clients();
    void read_client(Client &client);
    void write_client(Client &client);
    void handle_http(Client &client);
    void send_messages(Client &client, const std::vector<std::string> &messages);
    void queue_close(Client &client, std::uint16_t code, const std::string &reason);
    bool authorized(const ws::Request &request) const;

    ServerOptions options_;
    int listener_{-1};
    unsigned short port_{0};
    bool stopping_{false};
    std::vector<std::unique_ptr<Client>> clients_;
};

} // namespace sim
