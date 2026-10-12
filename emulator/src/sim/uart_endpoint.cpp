#include "sim/uart_endpoint.hpp"

namespace sim {

void UartEndpoint::set(const Value &value) {
    if (!can_receive())
        throw std::logic_error(name_ + " cannot receive data");
    if (const auto *bytes = std::get_if<Bytes>(&value))
        rx_pending_.insert(rx_pending_.end(), bytes->begin(), bytes->end());
    else if (const auto *text = std::get_if<std::string>(&value))
        rx_pending_.insert(rx_pending_.end(), text->begin(), text->end());
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
    while (!rx_pending_.empty() && mcu_.uart_rx(index_, rx_pending_.front()))
        rx_pending_.pop_front();
}

} // namespace sim
