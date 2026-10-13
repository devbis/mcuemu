#include "mcuemu/api.h"

#include "mcuemu/cpu.hpp"
#include "mcuemu/tlsr8258.hpp"

#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

struct mcuemu_engine {
    explicit mcuemu_engine(mcuemu::Tlsr8258Model model)
        : device(model), cpu(device) {}

    mcuemu::Tlsr8258 device;
    mcuemu::Cpu cpu;
    std::string error;
};

extern "C" {

mcuemu_engine *mcuemu_open(const char *model) {
    try {
        const auto parsed = mcuemu::parse_model(model ? model : "TLSR8258F512");
        return new mcuemu_engine(parsed);
    } catch (...) {
        return nullptr;
    }
}

void mcuemu_close(mcuemu_engine *engine) { delete engine; }

int mcuemu_load_raw(mcuemu_engine *engine, const char *path,
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

int mcuemu_run(mcuemu_engine *engine, std::size_t instruction_limit,
                mcuemu_run_result *result) {
    if (!engine || !result)
        return -1;
    const mcuemu::RunResult run = engine->cpu.run(instruction_limit);
    result->reason = run.reason == mcuemu::StopReason::Halted
                         ? MCUEMU_HALTED
                         : run.reason == mcuemu::StopReason::Fault ? MCUEMU_FAULT
                                                                    : MCUEMU_STEP_LIMIT;
    result->instructions = run.instructions;
    result->pc = run.pc;
    result->instruction = run.instruction;
    engine->error = run.message;
    return 0;
}

int mcuemu_reg_read(mcuemu_engine *engine, unsigned register_index,
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

int mcuemu_reg_write(mcuemu_engine *engine, unsigned register_index,
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

const char *mcuemu_last_error(const mcuemu_engine *engine) {
    return engine ? engine->error.c_str() : "invalid emulator handle";
}

} // extern "C"
