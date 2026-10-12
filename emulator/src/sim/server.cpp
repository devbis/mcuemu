#include "sim/server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace sim {

namespace {

constexpr std::size_t SoftOutputLimit = 4u << 20;  // stop advancing a slow reader
constexpr std::size_t HardOutputLimit = 64u << 20; // drop a client that never reads

void set_nonblocking(int fd) {
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

bool is_loopback(const std::string &host) {
    return host == "127.0.0.1" || host == "localhost" || host.rfind("127.", 0) == 0;
}

} // namespace

struct Server::Client {
    int fd{-1};
    std::string in, out;
    bool upgraded{false};
    bool closing{false}; // close once `out` is flushed
    std::unique_ptr<RpcSession> rpc;
    ws::FrameParser parser;
    std::chrono::steady_clock::time_point last_activity{std::chrono::steady_clock::now()};
};

Server::Server(ServerOptions options) : options_(std::move(options)) {
    if (!is_loopback(options_.bind) && options_.token.empty())
        throw std::invalid_argument("binding to a non-loopback address requires a token");
    RpcSession probe(options_.root); // validates the root early
    (void)probe;
    std::string host = options_.bind == "localhost" ? "127.0.0.1" : options_.bind;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(options_.port);
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
        throw std::invalid_argument("bind address must be an IPv4 address, got '" + host + "'");
    listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listener_ < 0)
        throw std::runtime_error("cannot create a socket");
    const int yes = 1;
    setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    if (::bind(listener_, reinterpret_cast<sockaddr *>(&addr), sizeof addr) != 0 ||
        ::listen(listener_, 16) != 0) {
        const std::string reason = std::strerror(errno);
        ::close(listener_);
        throw std::runtime_error("cannot listen on " + host + ":" +
                                 std::to_string(options_.port) + ": " + reason);
    }
    set_nonblocking(listener_);
    socklen_t length = sizeof addr;
    getsockname(listener_, reinterpret_cast<sockaddr *>(&addr), &length);
    port_ = ntohs(addr.sin_port);
}

Server::~Server() {
    for (auto &client : clients_)
        ::close(client->fd);
    if (listener_ >= 0)
        ::close(listener_);
}

bool Server::authorized(const ws::Request &request) const {
    if (options_.token.empty())
        return true;
    const std::string wanted = "token=" + options_.token;
    // The query may hold several parameters.
    std::size_t start = 0;
    while (start <= request.query.size()) {
        const std::size_t end = request.query.find('&', start);
        if (request.query.substr(start, end == std::string::npos ? std::string::npos : end - start) == wanted)
            return true;
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    const auto it = request.headers.find("authorization");
    return it != request.headers.end() && it->second == "Bearer " + options_.token;
}

void Server::handle_http(Client &client) {
    std::optional<ws::Request> request;
    try {
        request = ws::parse_request(client.in);
    } catch (const std::invalid_argument &error) {
        client.out += ws::http_response(400, "Bad Request", error.what());
        client.closing = true;
        return;
    }
    if (!request)
        return;
    if (request->method == "GET" && request->path == "/health" && !ws::is_upgrade(*request)) {
        client.out += ws::http_response(200, "OK", "ok\n");
        client.closing = true;
        return;
    }
    if (!ws::is_upgrade(*request)) {
        client.out += ws::http_response(426, "Upgrade Required", "this is a WebSocket endpoint\n");
        client.closing = true;
        return;
    }
    const auto origin = request->headers.find("origin");
    if (origin != request->headers.end()) {
        const auto &allowed = options_.allowed_origins;
        if (std::find(allowed.begin(), allowed.end(), origin->second) == allowed.end() &&
            std::find(allowed.begin(), allowed.end(), "*") == allowed.end()) {
            client.out += ws::http_response(403, "Forbidden", "origin not allowed\n");
            client.closing = true;
            return;
        }
    }
    if (!authorized(*request)) {
        client.out += ws::http_response(401, "Unauthorized", "a valid token is required\n");
        client.closing = true;
        return;
    }
    const auto sessions = std::count_if(clients_.begin(), clients_.end(),
                                        [](const auto &c) { return c->upgraded; });
    if (static_cast<std::size_t>(sessions) >= options_.max_sessions) {
        client.out += ws::http_response(503, "Service Unavailable", "too many sessions\n");
        client.closing = true;
        return;
    }
    client.rpc = std::make_unique<RpcSession>(options_.root);
    client.upgraded = true;
    client.out += ws::handshake_response(*request);
    // Bytes that followed the request head belong to the WebSocket stream.
    client.parser.feed(client.in.data(), client.in.size());
    client.in.clear();
}

void Server::send_messages(Client &client, const std::vector<std::string> &messages) {
    for (const std::string &message : messages)
        client.out += ws::encode_frame(ws::Opcode::Text, message);
}

void Server::queue_close(Client &client, std::uint16_t code, const std::string &reason) {
    client.out += ws::encode_close(code, reason);
    client.closing = true;
}

void Server::read_client(Client &client) {
    char buffer[65536];
    for (;;) {
        const ssize_t n = ::recv(client.fd, buffer, sizeof buffer, 0);
        if (n > 0) {
            client.last_activity = std::chrono::steady_clock::now();
            if (client.closing)
                continue; // drain and ignore
            if (!client.upgraded) {
                client.in.append(buffer, static_cast<std::size_t>(n));
                handle_http(client);
                if (!client.upgraded)
                    continue;
            } else {
                client.parser.feed(buffer, static_cast<std::size_t>(n));
            }
            try {
                while (auto message = client.parser.next()) {
                    switch (message->opcode) {
                    case ws::Opcode::Text:
                        send_messages(client, client.rpc->handle(message->payload));
                        if (client.rpc->close_requested())
                            queue_close(client, 1000, "bye");
                        break;
                    case ws::Opcode::Binary:
                        queue_close(client, 1003, "text messages only");
                        break;
                    case ws::Opcode::Ping:
                        client.out += ws::encode_frame(ws::Opcode::Pong, message->payload);
                        break;
                    case ws::Opcode::Pong:
                        break;
                    case ws::Opcode::Close:
                        client.out += ws::encode_frame(ws::Opcode::Close, message->payload);
                        client.closing = true;
                        break;
                    default:
                        break;
                    }
                    if (client.closing)
                        break;
                }
            } catch (const ws::ProtocolError &error) {
                queue_close(client, error.code(), error.what());
            }
        } else if (n == 0) {
            client.closing = true;
            client.out.clear(); // peer is gone
            return;
        } else {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                client.closing = true;
                client.out.clear();
            }
            return;
        }
    }
}

void Server::write_client(Client &client) {
    while (!client.out.empty()) {
#ifdef MSG_NOSIGNAL
        const ssize_t n = ::send(client.fd, client.out.data(), client.out.size(), MSG_NOSIGNAL);
#else
        const ssize_t n = ::send(client.fd, client.out.data(), client.out.size(), 0);
#endif
        if (n > 0) {
            client.out.erase(0, static_cast<std::size_t>(n));
        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;
        } else {
            client.out.clear();
            client.closing = true;
            return;
        }
    }
}

void Server::accept_clients() {
    for (;;) {
        const int fd = ::accept(listener_, nullptr, nullptr);
        if (fd < 0)
            return;
        set_nonblocking(fd);
#ifdef SO_NOSIGPIPE
        const int yes = 1;
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof yes);
#endif
        if (clients_.size() >= options_.max_sessions * 4) { // handshakes included
            ::close(fd);
            continue;
        }
        auto client = std::make_unique<Client>();
        client->fd = fd;
        clients_.push_back(std::move(client));
    }
}

void Server::run_once(int timeout_ms) {
    std::vector<pollfd> fds;
    fds.push_back({listener_, POLLIN, 0});
    bool busy = false;
    for (const auto &client : clients_) {
        short events = POLLIN;
        if (!client->out.empty())
            events |= POLLOUT;
        fds.push_back({client->fd, events, 0});
        busy |= client->upgraded && client->rpc->running() && client->out.size() < SoftOutputLimit;
    }
    ::poll(fds.data(), static_cast<nfds_t>(fds.size()), busy ? 0 : timeout_ms);

    if (fds[0].revents & POLLIN)
        accept_clients();
    for (std::size_t i = 0; i < fds.size() - 1 && i < clients_.size(); ++i) {
        Client &client = *clients_[i];
        if (fds[i + 1].fd != client.fd)
            continue;
        if (fds[i + 1].revents & (POLLIN | POLLHUP | POLLERR))
            read_client(client);
    }
    const auto now = std::chrono::steady_clock::now();
    for (auto &client : clients_) {
        if (client->upgraded && !client->closing && client->rpc->running() &&
            client->out.size() < SoftOutputLimit)
            send_messages(*client, client->rpc->pump(options_.pump_budget));
        if (client->out.size() > HardOutputLimit) {
            client->out.clear();
            client->closing = true;
        }
        if (!client->closing && !(client->upgraded && client->rpc->running()) &&
            now - client->last_activity > options_.idle_timeout)
            queue_close(*client, 1001, "idle timeout");
        write_client(*client);
    }
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(),
                                  [](const std::unique_ptr<Client> &client) {
                                      if (client->closing && client->out.empty()) {
                                          ::close(client->fd);
                                          return true;
                                      }
                                      return false;
                                  }),
                   clients_.end());
}

void Server::run() {
    while (!stopping_)
        run_once(100);
}

} // namespace sim
