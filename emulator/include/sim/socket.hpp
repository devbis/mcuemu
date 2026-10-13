#pragma once

// A thin layer over BSD sockets (POSIX) and WinSock, so the control server, the
// TCP sink and their tests share one code path. Only what mcuemu uses is
// wrapped: TCP over IPv4, non-blocking, polled.

#include <cstddef>
#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <chrono>
#include <thread>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>
#endif

namespace sim::sock {

#ifdef _WIN32
using socket_t = SOCKET;
inline constexpr socket_t invalid_socket = INVALID_SOCKET;
using PollFd = WSAPOLLFD;
using socket_length = int;
#else
using socket_t = int;
inline constexpr socket_t invalid_socket = -1;
using PollFd = pollfd;
using socket_length = socklen_t;
#endif

inline bool valid(socket_t s) { return s != invalid_socket; }

// WinSock must be started once per process; a no-op elsewhere.
inline void startup() {
#ifdef _WIN32
    static const bool started = [] {
        WSADATA data;
        return WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }();
    (void)started;
#endif
}

inline socket_t open_tcp() {
    startup();
    return ::socket(AF_INET, SOCK_STREAM, 0);
}

inline void close_socket(socket_t s) {
#ifdef _WIN32
    ::closesocket(s);
#else
    ::close(s);
#endif
}

inline void set_nonblocking(socket_t s) {
#ifdef _WIN32
    u_long on = 1;
    ::ioctlsocket(s, FIONBIO, &on);
#else
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
#endif
}

// A connection that must not raise SIGPIPE when the peer is gone.
inline void configure_client(socket_t s) {
    set_nonblocking(s);
#ifdef SO_NOSIGPIPE
    const int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof yes);
#endif
}

// SO_REUSEADDR, bind and listen. On failure the reason is in `reason`.
inline bool bind_listen(socket_t s, const sockaddr_in &address, int backlog,
                        std::string &reason) {
    const int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&yes),
               sizeof yes);
    if (::bind(s, reinterpret_cast<const sockaddr *>(&address), sizeof address) == 0 &&
        ::listen(s, backlog) == 0)
        return true;
#ifdef _WIN32
    reason = "WinSock error " + std::to_string(WSAGetLastError());
#else
    reason = std::strerror(errno);
#endif
    return false;
}

inline unsigned short local_port(socket_t s) {
    sockaddr_in address{};
    socket_length length = sizeof address;
    getsockname(s, reinterpret_cast<sockaddr *>(&address), &length);
    return ntohs(address.sin_port);
}

// send/recv return the byte count, 0 on an orderly close and -1 on error (then
// `would_block()` tells a full or empty non-blocking socket from a failure).
inline std::ptrdiff_t send_some(socket_t s, const char *data, std::size_t size) {
#ifdef _WIN32
    return ::send(s, data, static_cast<int>(size), 0);
#elif defined(MSG_NOSIGNAL)
    return ::send(s, data, size, MSG_NOSIGNAL);
#else
    return ::send(s, data, size, 0);
#endif
}

inline std::ptrdiff_t recv_some(socket_t s, void *data, std::size_t size) {
#ifdef _WIN32
    return ::recv(s, static_cast<char *>(data), static_cast<int>(size), 0);
#else
    return ::recv(s, data, size, 0);
#endif
}

inline bool would_block() {
#ifdef _WIN32
    return WSAGetLastError() == WSAEWOULDBLOCK;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
}

inline int poll(PollFd *fds, std::size_t count, int timeout_ms) {
#ifdef _WIN32
    return ::WSAPoll(fds, static_cast<ULONG>(count), timeout_ms);
#else
    return ::poll(fds, static_cast<nfds_t>(count), timeout_ms);
#endif
}

inline void sleep_us(unsigned microseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

} // namespace sim::sock
