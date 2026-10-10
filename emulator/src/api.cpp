#include "tc32emu/api.h"

#include "tc32emu/cpu.hpp"
#include "tc32emu/tlsr8258.hpp"

#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

struct tc32emu_engine {
    explicit tc32emu_engine(tc32emu::Tlsr8258Model model)
        : device(model), cpu(device) {}

    tc32emu::Tlsr8258 device;
    tc32emu::Cpu cpu;
    std::string error;
};

extern "C" {

tc32emu_engine *tc32emu_open(const char *model) {
    try {
        const auto parsed = tc32emu::parse_model(model ? model : "TLSR8258F512");
        return new tc32emu_engine(parsed);
    } catch (...) {
        return nullptr;
    }
}

void tc32emu_close(tc32emu_engine *engine) { delete engine; }

int tc32emu_load_raw(tc32emu_engine *engine, const char *path,
                     std::uint32_t flash_offset) {
    if (!engine || !path)
        return -1;
    try {
        std::ifstream input(path, std::ios::binary);
        if (!input)
            throw std::runtime_error("cannot open firmware image");
        std::vector<std::uint8_t> image(
            (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        engine->device.load_firmware(image, flash_offset);
        engine->cpu.sync_flash();
        engine->cpu.reset(0);
        engine->error.clear();
        return 0;
    } catch (const std::exception &error) {
        engine->error = error.what();
        return -1;
    }
}

int tc32emu_run(tc32emu_engine *engine, std::size_t instruction_limit,
                tc32emu_run_result *result) {
    if (!engine || !result)
        return -1;
    const tc32emu::RunResult run = engine->cpu.run(instruction_limit);
    result->reason = run.reason == tc32emu::StopReason::Halted
                         ? TC32EMU_HALTED
                         : run.reason == tc32emu::StopReason::Fault ? TC32EMU_FAULT
                                                                    : TC32EMU_STEP_LIMIT;
    result->instructions = run.instructions;
    result->pc = run.pc;
    result->instruction = run.instruction;
    engine->error = run.message;
    return 0;
}

int tc32emu_reg_read(tc32emu_engine *engine, unsigned register_index,
                     std::uint32_t *value) {
    if (!engine || !value || register_index > 15)
        return -1;
    try {
        *value = engine->cpu.reg(register_index);
        return 0;
    } catch (const std::exception &error) {
        engine->error = error.what();
        return -1;
    }
}

int tc32emu_reg_write(tc32emu_engine *engine, unsigned register_index,
                      std::uint32_t value) {
    if (!engine || register_index > 15)
        return -1;
    try {
        engine->cpu.set_reg(register_index, value);
        return 0;
    } catch (const std::exception &error) {
        engine->error = error.what();
        return -1;
    }
}

const char *tc32emu_last_error(const tc32emu_engine *engine) {
    return engine ? engine->error.c_str() : "invalid emulator handle";
}

} // extern "C"
