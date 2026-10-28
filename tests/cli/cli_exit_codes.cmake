# CLI exit codes and messages (run as `cmake -DMCUEMU=... -DWORK=... -P`).
# `run` and `sim` exit 1 when the firmware faults and 2 for usage errors.
if(NOT MCUEMU OR NOT WORK)
    message(FATAL_ERROR "MCUEMU and WORK are required")
endif()
file(MAKE_DIRECTORY "${WORK}")

function(expect_run expected_rc expected_text)
    execute_process(COMMAND "${MCUEMU}" ${ARGN}
                    RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT rc EQUAL expected_rc)
        message(FATAL_ERROR "mcuemu ${ARGN}: exit ${rc}, expected ${expected_rc}\n${out}${err}")
    endif()
    string(FIND "${out}${err}" "${expected_text}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "mcuemu ${ARGN}: output lacks '${expected_text}'\n${out}${err}")
    endif()
endfunction()

# 0x0000: NOP; 0x0001: 0xA5 (no MCS-51 instruction) -> Fault.
file(WRITE "${WORK}/fault.hex" ":0200000000A559\n:00000001FF\n")
# MOV PCON,#1 with no interrupt source -> Sleeping, not a fault.
file(WRITE "${WORK}/sleep.hex" ":0500000075870180FE80\n:00000001FF\n")
foreach(name fault sleep)
    file(WRITE "${WORK}/${name}.yaml"
         "version: 1\ndevices:\n  - id: d\n    mcus:\n"
         "      - {id: m, model: cc2530f256, firmware: \"${WORK}/${name}.hex\"}\n")
endforeach()

expect_run(1 "fault" sim "${WORK}/fault.yaml" --for 10us)
expect_run(0 "sleeping" sim "${WORK}/sleep.yaml" --for 10us)
expect_run(2 "TLSR8258F512, TLSR8258F1K, CA51F253L3, CC2530F256 or NRF52840"
           run --model NOSUCHMCU --firmware "${WORK}/fault.hex")
# INC 0x30; SJMP 0x0000: arrives at 0x0000 every two instructions.
file(WRITE "${WORK}/loop.hex" ":04000000053080FC4B\n:00000001FF\n")
expect_run(0 "until-pc-hits: 3" run --model CC2530F256 --firmware "${WORK}/loop.hex"
           --steps 100 --until-pc 0 --hits 3)
expect_run(0 "hit: 2 instructions=4 cycles=6 pc=0x0000" run --model CC2530F256
           --firmware "${WORK}/loop.hex" --steps 100 --until-pc 0 --hits 3 --report-hits)
expect_run(2 "--hits, --report-hits and --write-xdata need --until-pc"
           run --model CC2530F256 --firmware "${WORK}/loop.hex" --hits 2)
expect_run(2 "--hits, --report-hits and --write-xdata need --until-pc"
           run --model CC2530F256 --firmware "${WORK}/loop.hex" --report-hits)
expect_run(2 "--hits, --report-hits and --write-xdata need --until-pc"
           run --model CC2530F256 --firmware "${WORK}/loop.hex" --write-xdata 1:0x100:00)
expect_run(2 "is after the last arrival" run --model CC2530F256 --firmware "${WORK}/loop.hex"
           --until-pc 0 --hits 2 --write-xdata 3:0x100:00)
expect_run(2 "leaves XDATA SRAM" run --model CC2530F256 --firmware "${WORK}/loop.hex"
           --until-pc 0 --write-xdata 1:0x1fff:0000)
expect_run(2 "even number of hex digits" run --model CC2530F256 --firmware "${WORK}/loop.hex"
           --until-pc 0 --write-xdata 1:0x100:0)
# Written on arrival 2 after its report: arrival 2 shows 00, arrival 3 shows 5a.
expect_run(0 "xdata 0x0100: 5a" run --model CC2530F256 --firmware "${WORK}/loop.hex"
           --steps 100 --until-pc 0 --hits 3 --report-hits --dump-xdata 0x100:1
           --write-xdata 2:0x100:5a)
expect_run(2 "--report-hits" run --model CA51F253L3 --firmware "${WORK}/loop.hex"
           --report-hits)
expect_run(2 "NRF52840" run --firmware "${WORK}/fault.hex")
