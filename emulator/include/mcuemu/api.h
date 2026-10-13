#ifndef MCUEMU_API_H
#define MCUEMU_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mcuemu_engine mcuemu_engine;

typedef enum mcuemu_stop_reason {
    MCUEMU_STEP_LIMIT = 0,
    MCUEMU_HALTED = 1,
    MCUEMU_FAULT = 2
} mcuemu_stop_reason;

typedef struct mcuemu_run_result {
    mcuemu_stop_reason reason;
    size_t instructions;
    uint32_t pc;
    uint16_t instruction;
} mcuemu_run_result;

mcuemu_engine *mcuemu_open(const char *model);
void mcuemu_close(mcuemu_engine *engine);
int mcuemu_load_raw(mcuemu_engine *engine, const char *path,
                     uint32_t flash_offset);
int mcuemu_run(mcuemu_engine *engine, size_t instruction_limit,
                mcuemu_run_result *result);
int mcuemu_reg_read(mcuemu_engine *engine, unsigned register_index,
                     uint32_t *value);
int mcuemu_reg_write(mcuemu_engine *engine, unsigned register_index,
                      uint32_t value);
const char *mcuemu_last_error(const mcuemu_engine *engine);

#ifdef __cplusplus
}
#endif

#endif
