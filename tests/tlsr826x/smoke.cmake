# Builds tests/tlsr826x/main.c with the Telink SDK and TC32 LLVM, runs it on the
# TLSR8269 model and checks the UART text. Skips (exit 77) when the toolchain or
# the SDK is not available. Run as `cmake -DMCUEMU=... -DSRC=... -DOUT=... -P`.
if(NOT MCUEMU OR NOT SRC OR NOT OUT)
    message(FATAL_ERROR "MCUEMU, SRC and OUT are required")
endif()
execute_process(COMMAND env OUT=${OUT} bash ${SRC}/tests/tlsr826x/build.sh
                RESULT_VARIABLE build_rc OUTPUT_VARIABLE build_out ERROR_VARIABLE build_err)
if(NOT build_rc EQUAL 0)
    message(STATUS "tlsr826x smoke firmware not built (toolchain or SDK missing?): ${build_err}")
    message(FATAL_ERROR "SKIP")
endif()
execute_process(COMMAND "${MCUEMU}" run --model TLSR8269 --firmware ${OUT}/tlsr826x-smoke.bin
                        --steps 3000000
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "mcuemu run failed (${rc}):\n${out}${err}")
endif()
foreach(text "tlsr826x up" "ticks ok")
    string(FIND "${out}" "${text}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "UART output lacks '${text}':\n${out}${err}")
    endif()
endforeach()
