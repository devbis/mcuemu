#include "sim/rpc.hpp"

#include <emscripten/emscripten.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Json = nlohmann::json;

std::string response_buffer;
std::string error_buffer;

const char *encode_messages(const std::vector<std::string> &messages) {
    Json result = Json::array();
    for (const std::string &message : messages)
        result.push_back(Json::parse(message));
    response_buffer = result.dump();
    error_buffer.clear();
    return response_buffer.c_str();
}

sim::RpcSession *as_session(void *handle) {
    return static_cast<sim::RpcSession *>(handle);
}

} // namespace

// The JavaScript UI owns one session. Returned strings remain valid until the
// next exported call, so the bridge can decode them synchronously with ccall.
extern "C" {

EMSCRIPTEN_KEEPALIVE void *mcuemu_wasm_session_create(const char *root) {
    try {
        return new sim::RpcSession(root ? root : "/");
    } catch (const std::exception &error) {
        error_buffer = error.what();
        return nullptr;
    }
}

EMSCRIPTEN_KEEPALIVE void mcuemu_wasm_session_destroy(void *handle) {
    delete as_session(handle);
}

EMSCRIPTEN_KEEPALIVE const char *mcuemu_wasm_session_request(
    void *handle, const char *request) {
    try {
        if (!handle || !request)
            throw std::invalid_argument("session and request are required");
        return encode_messages(as_session(handle)->handle(request));
    } catch (const std::exception &error) {
        error_buffer = error.what();
        response_buffer = "[]";
        return response_buffer.c_str();
    }
}

EMSCRIPTEN_KEEPALIVE const char *mcuemu_wasm_session_pump(
    void *handle, int wall_budget_ms) {
    try {
        if (!handle)
            throw std::invalid_argument("session is required");
        const int budget = std::clamp(wall_budget_ms, 1, 20);
        return encode_messages(as_session(handle)->pump(
            std::chrono::milliseconds(budget)));
    } catch (const std::exception &error) {
        error_buffer = error.what();
        response_buffer = "[]";
        return response_buffer.c_str();
    }
}

EMSCRIPTEN_KEEPALIVE const char *mcuemu_wasm_last_error() {
    return error_buffer.c_str();
}

} // extern "C"
