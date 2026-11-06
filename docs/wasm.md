# Browser WebAssembly port

This is an early browser-facing build of the simulation control layer. The
WebAssembly module exposes the existing JSON-RPC methods to JavaScript; the
browser supplies configuration and firmware files through Emscripten's
virtual filesystem. No browser UI is included yet.

The MCU execution engines use Unicorn's TCG interpreter in this build, so the
browser does not need executable memory. The interpreter build is selected
automatically by Emscripten in the nested `engine/unicorn` repository.

## Build

Load the SDK environment in your shell (adjust the path if installed elsewhere):

```sh
source /path/to/emsdk/emsdk_env.sh
emcc --version
emcmake cmake -S . -B build-wasm -DMCUEMU_BUILD_WASM=ON
cmake --build build-wasm --target mcuemu-wasm
```

This produces `mcuemu-wasm.mjs` and `mcuemu-wasm.wasm`. Serve them over HTTP;
loading the generated module from `file://` is not supported by browsers.

## GitHub Actions

`.github/workflows/wasm.yml` builds the same target on pushes and pull requests
for `main` and `wasm`, and uploads the two files as the `mcuemu-wasm` artifact.
The workflow checks out `devbis/unicorn` branch `wasm`; push that branch to the
fork before running the workflow on GitHub.

## JavaScript bridge

Create one control session per module instance with
`mcuemu_wasm_session_create(root)`, send JSON-RPC messages with
`mcuemu_wasm_session_request(session, text)`, and call
`mcuemu_wasm_session_pump(session, budget_ms)` from the browser event loop while
a simulation is running. The pump budget is clamped to 1–20 ms. Both request
and pump return a JSON array of response and notification objects. Call
`mcuemu_wasm_session_destroy(session)` when the UI closes. A returned string is
valid only until the next exported call. `mcuemu_wasm_last_error()` reports
bridge errors; JSON-RPC errors remain in the returned messages.

Upload configuration dependencies and firmware bytes into the virtual
filesystem before calling `load`. The paths in YAML are resolved under the
session root, just as they are by the native control API.

The Emscripten build and MCU firmware execution path have been smoke-tested
with two TLSR8258 nodes. No performance claim is made for the interpreted
backend.
