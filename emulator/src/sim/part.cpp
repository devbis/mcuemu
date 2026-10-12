#include "sim/part.hpp"

#include <cctype>
#include <cmath>
#include <stdexcept>

namespace sim {

std::optional<std::string> ParamReader::string(const std::string &key) {
    const auto it = params_.find(key);
    if (it == params_.end())
        return std::nullopt;
    used_.insert(key);
    return it->second;
}

double ParamReader::number(const std::string &key, double fallback) {
    const auto text = string(key);
    if (!text)
        return fallback;
    std::size_t used = 0;
    double value = 0;
    try {
        value = std::stod(*text, &used);
    } catch (const std::exception &) {
        used = 0;
    }
    if (used != text->size() || !std::isfinite(value))
        throw std::invalid_argument("param '" + key + "' must be a number, got '" +
                                    *text + "'");
    return value;
}

std::int64_t ParamReader::integer(const std::string &key, std::int64_t fallback) {
    const auto text = string(key);
    if (!text)
        return fallback;
    std::size_t used = 0;
    long long value = 0;
    try {
        value = std::stoll(*text, &used, 0); // decimal or 0x hex
    } catch (const std::exception &) {
        used = 0;
    }
    if (used != text->size())
        throw std::invalid_argument("param '" + key +
                                    "' must be an integer, got '" + *text + "'");
    return value;
}

double ParamReader::resistance(const std::string &key, double fallback) {
    const auto text = string(key);
    if (!text)
        return fallback;
    std::string number = *text;
    double scale = 1.0;
    if (!number.empty()) {
        const char suffix =
            static_cast<char>(std::tolower(static_cast<unsigned char>(number.back())));
        if (suffix == 'k' || suffix == 'm') {
            // "M" and "m" both mean mega: milliohm resistors are not a thing
            // for thermistor dividers.
            scale = suffix == 'k' ? 1e3 : 1e6;
            number.pop_back();
        }
    }
    std::size_t used = 0;
    double value = 0;
    try {
        value = std::stod(number, &used);
    } catch (const std::exception &) {
        used = 0;
    }
    if (number.empty() || used != number.size() || !(value > 0))
        throw std::invalid_argument("param '" + key +
                                    "' must be a positive resistance (e.g. 4700, "
                                    "4.7k, 10k), got '" + *text + "'");
    return value * scale;
}

void ParamReader::finish() const {
    for (const auto &entry : params_)
        if (!used_.count(entry.first))
            throw std::invalid_argument("unknown param '" + entry.first + "'");
}

double value_as_number(const Value &value) {
    if (const auto *b = std::get_if<bool>(&value))
        return *b ? 1.0 : 0.0;
    if (const auto *i = std::get_if<std::int64_t>(&value))
        return static_cast<double>(*i);
    if (const auto *d = std::get_if<double>(&value))
        return *d;
    throw std::invalid_argument("a number is required");
}

void ParamEndpoint::set(const Value &value) {
    setter_(value);
    const EndpointEvent event{clock_(), name_, getter_()};
    const auto snapshot = sinks_;
    for (const auto &entry : snapshot)
        entry.second(event);
}

std::size_t ParamEndpoint::subscribe(EndpointSink sink) {
    const std::size_t id = next_id_++;
    sinks_.emplace(id, std::move(sink));
    return id;
}

void PartRegistry::add(std::string type, Factory factory) {
    factories_[std::move(type)] = std::move(factory);
}

const PartRegistry::Factory *PartRegistry::find(const std::string &type) const {
    const auto it = factories_.find(type);
    return it == factories_.end() ? nullptr : &it->second;
}

std::vector<std::string> PartRegistry::types() const {
    std::vector<std::string> names;
    for (const auto &entry : factories_)
        names.push_back(entry.first);
    return names;
}

} // namespace sim
