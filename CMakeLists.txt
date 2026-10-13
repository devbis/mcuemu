cmake_minimum_required(VERSION 3.20)
project(mcuemu VERSION 0.1.0 LANGUAGES C CXX)

# An unset build type compiles without optimisation, which makes the emulator
# about 5x slower; default to Release for single-config generators.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(MCUEMU_BUILD_TESTS "Build the emulator self-tests" ON)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/engine/unicorn/CMakeLists.txt")
    message(FATAL_ERROR
        "Unicorn source checkout is missing. Run scripts/bootstrap-unicorn.sh first.")
endif()

# The board wrapper depends on the TC32 architecture implemented in this local
# Unicorn checkout. Keep the engine as a separate Git repository under ./engine.
set(UNICORN_ARCH "tc32;mcs51;arm" CACHE STRING "Architectures built for device emulators" FORCE)
set(UNICORN_BUILD_TESTS OFF CACHE BOOL "Build Unicorn tests through the wrapper" FORCE)
set(UNICORN_INSTALL OFF CACHE BOOL "Install Unicorn through the wrapper" FORCE)
add_subdirectory(engine/unicorn EXCLUDE_FROM_ALL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    # <windows.h> (pulled in by WinSock) must not define min/max macros.
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
    set(MCUEMU_WARNING_FLAGS /W4)
    # A Tlsr8258 holds ~130 KiB of inline RAM and tests keep several on the
    # stack; the 1 MiB Windows default overflows, other platforms give 8 MiB.
    add_link_options(/STACK:8388608)
else()
    set(MCUEMU_WARNING_FLAGS -Wall -Wextra -Wpedantic)
endif()

add_library(mcuemu-core
    emulator/src/cpu.cpp
    emulator/src/tlsr8258.cpp
    emulator/src/tlsr8258_i2c.cpp
    emulator/src/tlsr8258_adc.cpp
    emulator/src/radio.cpp
    emulator/src/api.cpp
)
target_include_directories(mcuemu-core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(mcuemu-core PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(mcuemu-core PUBLIC unicorn)

add_executable(mcuemu emulator/src/main.cpp)
target_link_libraries(mcuemu PRIVATE mcuemu-core mcs51emu nrf52emu simcore)

add_library(mcs51emu
    emulator/src/mcs51/ca51f253l3.cpp
    emulator/src/mcs51/run_command.cpp
)
target_include_directories(mcs51emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(mcs51emu PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(mcs51emu PUBLIC unicorn)

# Generic Cortex-M core on the Unicorn fork's ARM backend (NVIC, SysTick, WFI/WFE).
add_library(cortexm emulator/src/cortexm/core.cpp)
target_include_directories(cortexm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(cortexm PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(cortexm PUBLIC unicorn)

add_library(nrf52emu
    emulator/src/nrf52/nrf52840.cpp
    emulator/src/nrf52/power_clock.cpp
    emulator/src/nrf52/nvmc.cpp
    emulator/src/nrf52/gpio.cpp
    emulator/src/nrf52/rtc.cpp
    emulator/src/nrf52/timer.cpp
    emulator/src/nrf52/misc.cpp
    emulator/src/nrf52/saadc.cpp
    emulator/src/nrf52/ecb.cpp
    emulator/src/nrf52/radio.cpp
    emulator/src/nrf52/uarte.cpp
    emulator/src/nrf52/twim.cpp
    emulator/src/nrf52/run_command.cpp
)
target_include_directories(nrf52emu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/emulator/src)
target_include_directories(nrf52emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(nrf52emu PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(nrf52emu PUBLIC cortexm mcuemu-core)

find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    include(FetchContent)
    # yaml-cpp 0.8.0 declares cmake_minimum_required(VERSION 3.4), which CMake 4
    # rejects; accept it instead of patching the dependency.
    if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
        set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    endif()
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0)
    FetchContent_MakeAvailable(yaml-cpp)
endif()
if(TARGET yaml-cpp::yaml-cpp)
    set(MCUEMU_YAML_TARGET yaml-cpp::yaml-cpp)
else()
    set(MCUEMU_YAML_TARGET yaml-cpp)
endif()

# Lua 5.4 for scripted parts, built from source so behavior does not depend on
# the system's Lua version. The upstream repository has no CMakeLists.txt.
include(FetchContent)
FetchContent_Declare(lua
    GIT_REPOSITORY https://github.com/lua/lua.git
    GIT_TAG v5.4.8)
FetchContent_MakeAvailable(lua)
file(GLOB MCUEMU_LUA_SOURCES "${lua_SOURCE_DIR}/*.c")
list(FILTER MCUEMU_LUA_SOURCES EXCLUDE REGEX "/(lua|luac|onelua|ltests)\\.c$")
add_library(lua54 STATIC ${MCUEMU_LUA_SOURCES})
target_include_directories(lua54 PUBLIC "${lua_SOURCE_DIR}")
set_target_properties(lua54 PROPERTIES C_STANDARD 99)
if(UNIX)
    target_compile_definitions(lua54 PRIVATE LUA_USE_POSIX)
    target_link_libraries(lua54 PUBLIC m)
endif()
target_compile_options(lua54 PRIVATE -w)

# JSON for the control API.
FetchContent_Declare(json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

# Chip-independent simulation core: scheduler, pins/nets, endpoints, world.
add_library(simcore
    emulator/src/sim/scheduler.cpp
    emulator/src/sim/net.cpp
    emulator/src/sim/endpoint.cpp
    emulator/src/sim/world.cpp
    emulator/src/sim/uart_endpoint.cpp
    emulator/src/sim/rtt_endpoint.cpp
    emulator/src/sim/sink.cpp
    emulator/src/sim/simulation.cpp
    emulator/src/sim/rpc.cpp
    emulator/src/sim/websocket.cpp
    emulator/src/sim/server.cpp
    emulator/src/sim/part.cpp
    emulator/src/sim/parts/ds18b20.cpp
    emulator/src/sim/parts/i2c_slave.cpp
    emulator/src/sim/parts/builtin.cpp
    emulator/src/sim/parts/lua_part.cpp
    emulator/src/sim/config.cpp
    emulator/src/sim/system.cpp
    emulator/src/sim/cli.cpp
    emulator/src/sim/tlsr8258_mcu.cpp
    emulator/src/sim/ca51f253l3_mcu.cpp
    emulator/src/sim/nrf52840_mcu.cpp
)
target_include_directories(simcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(simcore PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(simcore PUBLIC mcuemu-core mcs51emu nrf52emu ${MCUEMU_YAML_TARGET} lua54
    nlohmann_json::nlohmann_json)
if(WIN32)
    target_link_libraries(simcore PUBLIC ws2_32)
endif()

if(MCUEMU_BUILD_TESTS)
    enable_testing()
    add_executable(mcuemu-tests
        tests/test_main.cpp
        tests/tc32/test_cpu.cpp
        tests/machine/test_tlsr8258.cpp
        tests/machine/test_api.cpp
        tests/radio/test_radio.cpp
    )
    target_link_libraries(mcuemu-tests PRIVATE mcuemu-core)
    add_test(NAME mcuemu-tests COMMAND mcuemu-tests)

    add_executable(mcs51emu-tests
        tests/mcs51/test_ca51f253l3.cpp
    )
    target_link_libraries(mcs51emu-tests PRIVATE mcs51emu)
    add_test(NAME mcs51emu-tests COMMAND mcs51emu-tests)

    add_executable(nrf52emu-tests
        tests/nrf52/test_main.cpp
        tests/nrf52/test_cortexm.cpp
        tests/nrf52/test_chip.cpp
        tests/nrf52/test_radio.cpp
    )
    target_include_directories(nrf52emu-tests PRIVATE tests/nrf52)
    target_link_libraries(nrf52emu-tests PRIVATE nrf52emu)
    add_test(NAME nrf52emu-tests COMMAND nrf52emu-tests)

    add_executable(simcore-tests
        tests/sim/test_sim.cpp
        tests/sim/test_config.cpp
        tests/sim/test_parts.cpp
        tests/sim/test_periph.cpp
        tests/sim/test_lua.cpp
        tests/sim/test_radio_medium.cpp
        tests/sim/test_attach.cpp
        tests/sim/test_nrf52.cpp
        tests/sim/test_control.cpp
    )
    target_link_libraries(simcore-tests PRIVATE simcore)
    target_compile_definitions(simcore-tests PRIVATE
        MCUEMU_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME simcore-tests COMMAND simcore-tests)

    set(MCUEMU_SAMPLE_GW_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/samplegw_tb03f.bin")
    set(MCUEMU_SAMPLE_LIGHT_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/samplelight_tb03f.bin")
    if(EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}" AND
       EXISTS "${MCUEMU_SAMPLE_LIGHT_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zigbee_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zigbee_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_SAMPLE_GW_FIRMWARE}"
                        "${MCUEMU_SAMPLE_LIGHT_FIRMWARE}"
                        join)
            add_test(
                NAME firmware_zigbee_button_light
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zigbee_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_SAMPLE_GW_FIRMWARE}"
                        "${MCUEMU_SAMPLE_LIGHT_FIRMWARE}"
                        button)
            set_tests_properties(firmware_zigbee_join firmware_zigbee_button_light
                PROPERTIES
                TIMEOUT 600
                LABELS "firmware;zigbee"
            )
        else()
            message(STATUS
                "Skipping firmware_zigbee_join: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_zigbee_join: local sampleGW/sampleLight images not found in firmware/")
    endif()

    set(MCUEMU_ZEPHYR_PM_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-pmtest.bin")
    if(EXISTS "${MCUEMU_ZEPHYR_PM_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zephyr_pm_wake
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zephyr_pm_wake.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_ZEPHYR_PM_FIRMWARE}")
            set_tests_properties(firmware_zephyr_pm_wake PROPERTIES
                TIMEOUT 180
                LABELS "firmware;power"
            )
        else()
            message(STATUS
                "Skipping firmware_zephyr_pm_wake: Python 3 interpreter not found")
        endif()
    endif()

    set(MCUEMU_NRF52840_SENSOR_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/nrf52840-sensor.bin")
    if(EXISTS "${MCUEMU_NRF52840_SENSOR_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_nrf52840_boot
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_nrf52840_boot.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_NRF52840_SENSOR_FIRMWARE}")
            set_tests_properties(firmware_nrf52840_boot PROPERTIES
                TIMEOUT 120
                LABELS "firmware"
            )
        else()
            message(STATUS
                "Skipping firmware_nrf52840_boot: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_nrf52840_boot: local nrf52840-sensor.bin not found in firmware/")
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/nrf52840-sensor-sht31.bin" AND
       EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_nrf52840_zigbee_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_nrf52840_zigbee_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${CMAKE_CURRENT_SOURCE_DIR}/examples/nrf52840_zigbee.yaml")
            set_tests_properties(firmware_nrf52840_zigbee_join PROPERTIES
                TIMEOUT 300
                LABELS "firmware;zigbee"
            )
        endif()
    endif()

    set(MCUEMU_VENDOR_SLEEP_1S_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/vendor_sleep_1s.bin")
    if(EXISTS "${MCUEMU_VENDOR_SLEEP_1S_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_vendor_sleep_1s
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_vendor_sleep_1s.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_VENDOR_SLEEP_1S_FIRMWARE}")
            set_tests_properties(firmware_vendor_sleep_1s PROPERTIES
                TIMEOUT 60
                LABELS "firmware;power"
            )
        else()
            message(STATUS
                "Skipping firmware_vendor_sleep_1s: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_vendor_sleep_1s: local vendor_sleep_1s.bin not found in firmware/")
    endif()
endif()
