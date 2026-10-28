#include "sim/uart_endpoint.hpp"

#include <cctype>
#include <cstdio>

namespace sim {

void UartEndpoint::set(const Value &value) {
    if (!can_receive())
        throw std::logic_error(name_ + " cannot receive data");
    if (const auto *bytes = std::get_if<Bytes>(&value))
        for (const std::uint8_t byte : *bytes)
            rx_pending_.push_back(UartFrame{.data = byte});
    else if (const auto *text = std::get_if<std::string>(&value))
        for (const char ch : *text)
            rx_pending_.push_back(UartFrame{.data = static_cast<std::uint8_t>(ch)});
    else
        throw std::invalid_argument("a UART takes bytes or a string");
}

std::size_t UartEndpoint::subscribe(EndpointSink sink) {
    const std::size_t id = next_id_++;
    sinks_.emplace(id, std::move(sink));
    return id;
}

void UartEndpoint::poll(std::uint64_t now_ns) {
    Bytes sent = mcu_.uart_take_tx(index_);
    if (!sent.empty()) {
        sent_ += sent.size();
        const EndpointEvent event{now_ns, name_, Value{std::move(sent)}};
        const auto snapshot = sinks_;
        for (const auto &entry : snapshot)
            entry.second(event);
    }
    // Plain frames take the byte path, exactly as before frames existed.
    while (!rx_pending_.empty()) {
        const UartFrame &frame = rx_pending_.front();
        const bool taken = frame.plain() ? mcu_.uart_rx(index_, frame.data)
                                         : mcu_.uart_rx_frame(index_, frame);
        if (!taken)
            break;
        rx_pending_.pop_front();
    }
}

void UartFramesEndpoint::set(const Value &value) {
    if (!can_receive())
        throw std::logic_error(name_ + " cannot receive data");
    std::vector<UartFrame> frames;
    if (const auto *text = std::get_if<std::string>(&value)) {
        frames = parse_uart_frames(*text);
    } else if (const auto *number = std::get_if<std::int64_t>(&value)) {
        if (*number < 0 || *number > 0xff)
            throw std::invalid_argument("a UART frame byte is 0..255");
        frames.push_back(UartFrame{.data = static_cast<std::uint8_t>(*number)});
    } else if (const auto *bytes = std::get_if<Bytes>(&value)) {
        for (const std::uint8_t byte : *bytes)
            frames.push_back(UartFrame{.data = byte});
    } else {
        throw std::invalid_argument("UART frames take a string of frame tokens, a byte or bytes");
    }
    for (const UartFrame &frame : frames)
        bytes_.queue(frame);
}

std::size_t UartFramesEndpoint::subscribe(EndpointSink sink) {
    if (sinks_.empty())
        mcu_.uart_capture_tx_frames(index_, true);
    const std::size_t id = next_id_++;
    sinks_.emplace(id, std::move(sink));
    return id;
}

void UartFramesEndpoint::unsubscribe(std::size_t id) {
    if (sinks_.erase(id) && sinks_.empty())
        mcu_.uart_capture_tx_frames(index_, false);
}

void UartFramesEndpoint::poll(std::uint64_t now_ns) {
    if (sinks_.empty())
        return;
    const std::vector<UartFrame> frames = mcu_.uart_take_tx_frames(index_);
    if (frames.empty())
        return;
    sent_ += frames.size();
    const EndpointEvent event{now_ns, name_, Value{format_uart_frames(frames)}};
    const auto snapshot = sinks_;
    for (const auto &entry : snapshot)
        entry.second(event);
}

namespace {

bool is_separator(char ch) { return ch == ',' || std::isspace(static_cast<unsigned char>(ch)); }

UartFrame parse_frame_token(const std::string &token) {
    const auto bad = [&token](const std::string &why) {
        return std::invalid_argument("UART frame '" + token + "': " + why);
    };
    if (token == "break")
        return UartFrame{.line_break = true};
    std::size_t colon = token.find(':');
    const std::string number = token.substr(0, colon);
    unsigned long value = 0;
    std::size_t used = 0;
    if (number.empty() || !std::isdigit(static_cast<unsigned char>(number[0])))
        throw bad("expected 'break' or a byte");
    try {
        value = std::stoul(number, &used, 0);
    } catch (const std::exception &) {
        throw bad("expected 'break' or a byte");
    }
    if (used != number.size() || value > 0xff)
        throw bad("the byte must be 0..255");
    UartFrame frame{.data = static_cast<std::uint8_t>(value)};
    bool ninth_given = false;
    while (colon != std::string::npos) {
        const std::size_t next = token.find(':', colon + 1);
        const std::string flag = token.substr(colon + 1, next == std::string::npos
                                                             ? std::string::npos
                                                             : next - colon - 1);
        if (flag == "9=0" || flag == "9=1" || flag == "perr") {
            if (ninth_given)
                throw bad("more than one 9th-bit flag (9=0, 9=1, perr)");
            ninth_given = true;
            frame.ninth = flag == "perr" ? UartFrame::Ninth::BadParity
                          : flag == "9=1" ? UartFrame::Ninth::One
                                          : UartFrame::Ninth::Zero;
        } else if (flag == "fe" || flag == "fe2") {
            bool &error = flag == "fe" ? frame.stop1_error : frame.stop2_error;
            if (error)
                throw bad("flag '" + flag + "' given twice");
            error = true;
        } else {
            throw bad("unknown flag '" + flag + "' (expected 9=0, 9=1, perr, fe or fe2)");
        }
        colon = next;
    }
    return frame;
}

} // namespace

std::vector<UartFrame> parse_uart_frames(std::string_view text) {
    std::vector<UartFrame> frames;
    std::size_t pos = 0;
    while (pos < text.size()) {
        if (is_separator(text[pos])) {
            ++pos;
            continue;
        }
        std::size_t end = pos;
        while (end < text.size() && !is_separator(text[end]))
            ++end;
        frames.push_back(parse_frame_token(std::string(text.substr(pos, end - pos))));
        pos = end;
    }
    return frames;
}

std::string format_uart_frames(const std::vector<UartFrame> &frames) {
    std::string out;
    for (const UartFrame &frame : frames) {
        if (!out.empty())
            out += ' ';
        if (frame.line_break) {
            out += "break";
            continue;
        }
        char byte[8];
        std::snprintf(byte, sizeof byte, "0x%02x", frame.data);
        out += byte;
        switch (frame.ninth) {
        case UartFrame::Ninth::Expected: break;
        case UartFrame::Ninth::Zero: out += ":9=0"; break;
        case UartFrame::Ninth::One: out += ":9=1"; break;
        case UartFrame::Ninth::BadParity: out += ":perr"; break;
        }
        if (frame.stop1_error)
            out += ":fe";
        if (frame.stop2_error)
            out += ":fe2";
    }
    return out;
}

} // namespace sim
