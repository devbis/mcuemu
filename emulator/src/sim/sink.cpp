#include "sim/sink.hpp"

#include "sim/config.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace sim {

namespace {

std::string value_text(const Value &value) {
    if (const auto *b = std::get_if<bool>(&value))
        return *b ? "1" : "0";
    if (const auto *i = std::get_if<std::int64_t>(&value))
        return std::to_string(*i);
    if (const auto *d = std::get_if<double>(&value)) {
        char buffer[64];
        std::snprintf(buffer, sizeof buffer, "%g", *d);
        return buffer;
    }
    if (const auto *s = std::get_if<std::string>(&value))
        return *s;
    if (const auto *bytes = std::get_if<Bytes>(&value)) {
        std::string text;
        static const char digits[] = "0123456789abcdef";
        for (std::uint8_t byte : *bytes) {
            text.push_back(digits[byte >> 4]);
            text.push_back(digits[byte & 15]);
        }
        return text;
    }
    return "";
}

// Bytes go out raw (a console wants characters); anything else is a line.
std::string render(const EndpointEvent &event) {
    if (const auto *bytes = std::get_if<Bytes>(&event.value))
        return std::string(bytes->begin(), bytes->end());
    return std::to_string(event.time_ns) + " " + event.endpoint + " = " +
           value_text(event.value) + "\n";
}

class StdoutSink final : public Sink {
public:
    void consume(const EndpointEvent &event) override {
        const std::string text = render(event);
        std::cout.write(text.data(), static_cast<std::streamsize>(text.size()));
        std::cout.flush();
    }
};

} // namespace

void HexLog::flush(const Pending &pending) {
    static const char digits[] = "0123456789abcdef";
    std::string line;
    char stamp[48];
    std::snprintf(stamp, sizeof stamp, "[%11.6f s] ",
                  static_cast<double>(pending.first_ns) / 1e9);
    line = stamp;
    line += pending.endpoint;
    line += "  ";
    line += std::to_string(pending.bytes.size());
    line += pending.bytes.size() == 1 ? " byte:" : " bytes:";
    for (std::uint8_t byte : pending.bytes) {
        line.push_back(' ');
        line.push_back(digits[byte >> 4]);
        line.push_back(digits[byte & 15]);
    }
    line.push_back('\n');
    std::cout.write(line.data(), static_cast<std::streamsize>(line.size()));
    std::cout.flush();
}

void HexLog::add(const EndpointEvent &event) {
    const auto *bytes = std::get_if<Bytes>(&event.value);
    if (!bytes || bytes->empty())
        return;
    // Everything that has been quiet for a gap is complete; print it oldest first.
    for (auto it = pending_.begin(); it != pending_.end();) {
        if (event.time_ns > it->last_ns && event.time_ns - it->last_ns > gap_ns_) {
            flush(*it);
            it = pending_.erase(it);
        } else {
            ++it;
        }
    }
    for (Pending &pending : pending_) {
        if (pending.endpoint == event.endpoint) {
            pending.bytes.insert(pending.bytes.end(), bytes->begin(), bytes->end());
            pending.last_ns = event.time_ns;
            return;
        }
    }
    pending_.push_back({event.endpoint, event.time_ns, event.time_ns, *bytes});
}

void HexLog::flush_all() {
    std::sort(pending_.begin(), pending_.end(),
              [](const Pending &a, const Pending &b) { return a.first_ns < b.first_ns; });
    for (const Pending &pending : pending_)
        flush(pending);
    pending_.clear();
}

namespace {

class HexSink final : public Sink {
public:
    explicit HexSink(std::shared_ptr<HexLog> log) : log_(std::move(log)) {}
    ~HexSink() override { log_->flush_all(); }
    void consume(const EndpointEvent &event) override { log_->add(event); }
    void finish() override { log_->flush_all(); }

private:
    std::shared_ptr<HexLog> log_;
};

class FileSink final : public Sink {
public:
    explicit FileSink(const std::string &path) : out_(path, std::ios::binary) {
        if (!out_)
            throw std::invalid_argument("cannot write '" + path + "'");
    }
    void consume(const EndpointEvent &event) override {
        const std::string text = render(event);
        out_.write(text.data(), static_cast<std::streamsize>(text.size()));
        out_.flush();
    }
    void finish() override { out_.flush(); }

private:
    std::ofstream out_;
};

class VcdSink final : public Sink {
public:
    VcdSink(std::shared_ptr<VcdWriter> writer, std::size_t signal)
        : writer_(std::move(writer)), signal_(signal) {}
    void consume(const EndpointEvent &event) override {
        writer_->record(signal_, event.time_ns, event.value);
    }
    void finish() override { writer_->write(); }

private:
    std::shared_ptr<VcdWriter> writer_;
    std::size_t signal_;
};

// Streams frames of one protocol into a pcap file, timestamped with the
// frame's start in virtual time. The air observer cannot be removed, so the
// sink is guarded by a flag that outlives it.
class PcapSink final : public Sink {
public:
    PcapSink(const std::string &path, tc32emu::VirtualAir &air,
             tc32emu::RadioProtocol protocol)
        : out_(path, std::ios::binary), protocol_(protocol),
          alive_(std::make_shared<bool>(true)) {
        if (!out_)
            throw std::invalid_argument("cannot write '" + path + "'");
        // LINKTYPE_IEEE802_15_4_NOFCS (230) or LINKTYPE_BLUETOOTH_LE_LL (251).
        put32(0xa1b2c3d4u);
        put16(2);
        put16(4);
        put32(0);
        put32(0);
        put32(65535);
        put32(protocol == tc32emu::RadioProtocol::Zigbee802154 ? 230u : 251u);
        out_.flush();
        auto alive = alive_;
        air.add_observer([this, alive](const tc32emu::RadioFrame &frame) {
            if (*alive)
                write_frame(frame);
        });
    }
    ~PcapSink() override { *alive_ = false; }
    // Frames arrive through the air observer; endpoint events are ignored.
    void consume(const EndpointEvent &) override {}
    void finish() override { out_.flush(); }

private:
    void put16(std::uint16_t v) { out_.write(reinterpret_cast<const char *>(&v), 2); }
    void put32(std::uint32_t v) { out_.write(reinterpret_cast<const char *>(&v), 4); }
    void write_frame(const tc32emu::RadioFrame &frame) {
        if (frame.protocol != protocol_)
            return;
        std::vector<std::uint8_t> data;
        if (protocol_ == tc32emu::RadioProtocol::Ble)
            data = {0xd6, 0xbe, 0x89, 0x8e}; // advertising access address, LE
        data.insert(data.end(), frame.payload.begin(), frame.payload.end());
        put32(static_cast<std::uint32_t>(frame.start_ns / 1000000000ull));
        put32(static_cast<std::uint32_t>(frame.start_ns % 1000000000ull / 1000));
        put32(static_cast<std::uint32_t>(data.size()));
        put32(static_cast<std::uint32_t>(data.size()));
        out_.write(reinterpret_cast<const char *>(data.data()),
                   static_cast<std::streamsize>(data.size()));
        out_.flush();
    }
    std::ofstream out_;
    tc32emu::RadioProtocol protocol_;
    std::shared_ptr<bool> alive_;
};

void set_nonblocking(int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

} // namespace

// ---- VCD ---------------------------------------------------------------

std::size_t VcdWriter::add_signal(const std::string &name, bool real,
                                  const Value &initial) {
    signals_.push_back({name, real, initial});
    return signals_.size() - 1;
}

void VcdWriter::record(std::size_t signal, std::uint64_t time_ns, const Value &value) {
    changes_.push_back({time_ns, signal, value});
}

void VcdWriter::write() {
    if (written_)
        return;
    written_ = true;
    std::ofstream out(path_);
    if (!out)
        throw std::runtime_error("cannot write '" + path_ + "'");
    auto identifier = [](std::size_t index) {
        std::string id;
        do {
            id.push_back(static_cast<char>('!' + index % 94));
            index /= 94;
        } while (index != 0);
        return id;
    };
    auto sample = [&](const Signal &signal, const Value &value, std::size_t index) {
        if (signal.real)
            return "r" + value_text(std::holds_alternative<std::monostate>(value) ? Value{0.0}
                                                                                   : value) +
                   " " + identifier(index);
        const auto *flag = std::get_if<bool>(&value);
        return std::string(flag && *flag ? "1" : "0") + identifier(index);
    };
    out << "$timescale 1 ns $end\n$scope module sim $end\n";
    for (std::size_t i = 0; i < signals_.size(); ++i) {
        std::string name = signals_[i].name;
        std::replace(name.begin(), name.end(), '.', '_');
        out << "$var " << (signals_[i].real ? "real 64 " : "wire 1 ") << identifier(i)
            << " " << name << " $end\n";
    }
    out << "$upscope $end\n$enddefinitions $end\n#0\n$dumpvars\n";
    for (std::size_t i = 0; i < signals_.size(); ++i)
        out << sample(signals_[i], signals_[i].initial, i) << "\n";
    out << "$end\n";
    std::stable_sort(changes_.begin(), changes_.end(),
                     [](const Change &a, const Change &b) { return a.time < b.time; });
    std::uint64_t current = 0;
    for (const Change &change : changes_) {
        if (change.time != current) {
            out << "#" << change.time << "\n";
            current = change.time;
        }
        out << sample(signals_[change.signal], change.value, change.signal) << "\n";
    }
}

// ---- TCP ---------------------------------------------------------------

TcpSink::TcpSink(const std::string &address, Endpoint &endpoint)
    : endpoint_(endpoint) {
    std::string host = "127.0.0.1";
    std::string port_text = address;
    if (const std::size_t colon = address.rfind(':'); colon != std::string::npos) {
        host = address.substr(0, colon);
        port_text = address.substr(colon + 1);
        if (host == "localhost")
            host = "127.0.0.1";
    }
    unsigned long port = 0;
    std::size_t used = 0;
    try {
        port = std::stoul(port_text, &used);
    } catch (const std::exception &) {
        used = 0;
    }
    if (port_text.empty() || used != port_text.size() || port > 65535)
        throw std::invalid_argument("tcp sink needs a port 0..65535, got '" +
                                    port_text + "'");
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<unsigned short>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
        throw std::invalid_argument("tcp sink needs an IPv4 address, got '" + host + "'");

    listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listener_ < 0)
        throw std::runtime_error("cannot create a socket");
    const int yes = 1;
    setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    if (::bind(listener_, reinterpret_cast<sockaddr *>(&addr), sizeof addr) != 0 ||
        ::listen(listener_, 1) != 0) {
        const std::string reason = std::strerror(errno);
        ::close(listener_);
        throw std::invalid_argument("tcp sink cannot listen on " + host + ":" +
                                    port_text + ": " + reason);
    }
    set_nonblocking(listener_);
    socklen_t length = sizeof addr;
    getsockname(listener_, reinterpret_cast<sockaddr *>(&addr), &length);
    port_ = ntohs(addr.sin_port);
}

TcpSink::~TcpSink() {
    drop_client();
    if (listener_ >= 0)
        ::close(listener_);
}

void TcpSink::drop_client() {
    if (client_ >= 0) {
        ::close(client_);
        client_ = -1;
    }
}

void TcpSink::consume(const EndpointEvent &event) {
    if (client_ < 0)
        return;
    const std::string text = render(event);
    std::size_t sent = 0;
    while (sent < text.size()) {
#ifdef MSG_NOSIGNAL
        const ssize_t n = ::send(client_, text.data() + sent, text.size() - sent, MSG_NOSIGNAL);
#else
        const ssize_t n = ::send(client_, text.data() + sent, text.size() - sent, 0);
#endif
        if (n > 0) {
            sent += static_cast<std::size_t>(n);
        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return; // a client that does not read loses data rather than stalling
        } else {
            drop_client();
            return;
        }
    }
}

void TcpSink::poll() {
    // A new connection replaces the old one.
    const int fresh = ::accept(listener_, nullptr, nullptr);
    if (fresh >= 0) {
        drop_client();
        client_ = fresh;
        set_nonblocking(client_);
#ifdef SO_NOSIGPIPE
        const int yes = 1;
        setsockopt(client_, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof yes);
#endif
    }
    if (client_ < 0)
        return;
    std::uint8_t buffer[512];
    for (;;) {
        const ssize_t n = ::recv(client_, buffer, sizeof buffer, 0);
        if (n > 0) {
            try {
                endpoint_.set(Value{Bytes(buffer, buffer + n)});
            } catch (const std::exception &error) {
                if (!warned_) {
                    std::cerr << "warning: tcp input dropped: " << error.what() << '\n';
                    warned_ = true;
                }
            }
        } else if (n == 0) {
            drop_client();
            return;
        } else {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                drop_client();
            return;
        }
    }
}

// ---- session -----------------------------------------------------------

Session::~Session() {
    try {
        finish();
    } catch (const std::exception &error) {
        std::cerr << "warning: " << error.what() << '\n';
    }
    for (Attachment &attachment : attachments_)
        attachment.endpoint->unsubscribe(attachment.subscription);
}

std::size_t Session::attach(const std::string &endpoint_name, const std::string &spec) {
    Endpoint &endpoint = registry_.at(endpoint_name);
    const std::size_t colon = spec.find(':');
    const std::string kind = spec.substr(0, colon);
    const std::string arg = colon == std::string::npos ? "" : spec.substr(colon + 1);
    std::unique_ptr<Sink> sink;
    if (kind == "stdout" && colon == std::string::npos) {
        sink = std::make_unique<StdoutSink>();
    } else if (kind == "hex") {
        // hex or hex:GAP (a duration such as 500us); the default gap is 1 ms.
        const std::uint64_t gap = arg.empty() ? 1000000u : parse_duration_ns(arg);
        auto &log = hex_logs_[gap];
        if (!log)
            log = std::make_shared<HexLog>(gap);
        sink = std::make_unique<HexSink>(log);
    } else if (kind == "file" && !arg.empty()) {
        sink = std::make_unique<FileSink>(arg);
    } else if (kind == "tcp" && !arg.empty()) {
        sink = std::make_unique<TcpSink>(arg, endpoint);
    } else if (kind == "vcd" && !arg.empty()) {
        const Value initial = endpoint.get();
        const bool real = std::holds_alternative<double>(initial) ||
                          std::holds_alternative<std::int64_t>(initial);
        if ((endpoint.kind() != "net" && endpoint.kind() != "param") ||
            (!real && !std::holds_alternative<bool>(initial)))
            throw std::invalid_argument("vcd needs a boolean or numeric endpoint; '" +
                                        endpoint_name + "' is a " + endpoint.kind());
        auto &writer = vcd_files_[arg];
        if (!writer)
            writer = std::make_shared<VcdWriter>(arg);
        sink = std::make_unique<VcdSink>(
            writer, writer->add_signal(endpoint_name, real, initial));
    } else if (kind == "pcap" && !arg.empty()) {
        auto *radio = dynamic_cast<RadioEndpoint *>(&endpoint);
        if (!radio)
            throw std::invalid_argument("pcap needs a radio endpoint; '" + endpoint_name +
                                        "' is a " + endpoint.kind());
        std::string path = arg;
        tc32emu::RadioProtocol protocol = tc32emu::RadioProtocol::Zigbee802154;
        if (const std::size_t comma = arg.rfind(','); comma != std::string::npos) {
            path = arg.substr(0, comma);
            const std::string option = arg.substr(comma + 1);
            if (option == "ble")
                protocol = tc32emu::RadioProtocol::Ble;
            else if (option != "zigbee")
                throw std::invalid_argument("pcap option must be zigbee or ble, got '" +
                                            option + "'");
        }
        sink = std::make_unique<PcapSink>(path, radio->air(), protocol);
    } else {
        throw std::invalid_argument(
            "unknown sink '" + spec +
            "' (use stdout, hex[:GAP], file:PATH, tcp:PORT, vcd:PATH or pcap:PATH[,ble])");
    }
    Sink *raw = sink.get();
    const std::size_t subscription =
        endpoint.subscribe([raw](const EndpointEvent &event) { raw->consume(event); });
    const std::size_t id = next_id_++;
    attachments_.push_back({id, &endpoint, subscription, std::move(sink)});
    return id;
}

bool Session::detach(std::size_t id) {
    const auto it = std::find_if(attachments_.begin(), attachments_.end(),
                                 [id](const Attachment &a) { return a.id == id; });
    if (it == attachments_.end())
        return false;
    it->endpoint->unsubscribe(it->subscription);
    it->sink->finish();
    attachments_.erase(it);
    return true;
}

void Session::poll() {
    for (Attachment &attachment : attachments_)
        attachment.sink->poll();
}

void Session::finish() {
    for (Attachment &attachment : attachments_)
        attachment.sink->finish();
}

Sink *Session::sink(std::size_t id) {
    for (Attachment &attachment : attachments_)
        if (attachment.id == id)
            return attachment.sink.get();
    return nullptr;
}

} // namespace sim
