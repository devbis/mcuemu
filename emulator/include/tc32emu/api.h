#ifndef TC32EMU_API_H
#define TC32EMU_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tc32emu_engine tc32emu_engine;

typedef enum tc32emu_stop_reason {
    TC32EMU_STEP_LIMIT = 0,
    TC32EMU_HALTED = 1,
    TC32EMU_FAULT = 2
} tc32emu_stop_reason;

typedef struct tc32emu_run_result {
    tc32emu_stop_reason reason;
    size_t instructions;
    uint32_t pc;
    uint16_t instruction;
} tc32emu_run_result;

tc32emu_engine *tc32emu_open(const char *model);
void tc32emu_close(tc32emu_engine *engine);
int tc32emu_load_raw(tc32emu_engine *engine, const char *path,
                     uint32_t flash_offset);
int tc32emu_run(tc32emu_engine *engine, size_t instruction_limit,
                tc32emu_run_result *result);
int tc32emu_reg_read(tc32emu_engine *engine, unsigned register_index,
                     uint32_t *value);
int tc32emu_reg_write(tc32emu_engine *engine, unsigned register_index,
                      uint32_t value);
const char *tc32emu_last_error(const tc32emu_engine *engine);

#ifdef __cplusplus
}
#endif

#endif
