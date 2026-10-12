cmake_minimum_required(VERSION 3.20)
project(tc32_emu VERSION 0.1.0 LANGUAGES C CXX)

option(TC32EMU_BUILD_TESTS "Build the emulator self-tests" ON)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/engine/unicorn/CMakeLists.txt")
    message(FATAL_ERROR
        "Unicorn source checkout is missing. Run scripts/bootstrap-unicorn.sh first.")
endif()

# The board wrapper depends on the TC32 architecture implemented in this local
# Unicorn checkout. Keep the engine as a separate Git repository under ./engine.
set(UNICORN_ARCH "tc32;mcs51" CACHE STRING "Architectures built for device emulators" FORCE)
set(UNICORN_BUILD_TESTS OFF CACHE BOOL "Build Unicorn tests through the wrapper" FORCE)
set(UNICORN_INSTALL OFF CACHE BOOL "Install Unicorn through the wrapper" FORCE)
add_subdirectory(engine/unicorn EXCLUDE_FROM_ALL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(tc32emu
    emulator/src/cpu.cpp
    emulator/src/tlsr8258.cpp
    emulator/src/tlsr8258_i2c.cpp
    emulator/src/tlsr8258_adc.cpp
    emulator/src/radio.cpp
    emulator/src/api.cpp
)
target_include_directories(tc32emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(tc32emu PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(tc32emu PUBLIC unicorn)

add_executable(tc32-emu emulator/src/main.cpp)
target_link_libraries(tc32-emu PRIVATE tc32emu simcore)

add_library(mcs51emu
    emulator/src/mcs51/ca51f253l3.cpp
)
target_include_directories(mcs51emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(mcs51emu PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mcs51emu PUBLIC unicorn)

add_executable(mcs51-emu emulator/src/mcs51/main.cpp)
target_link_libraries(mcs51-emu PRIVATE mcs51emu)

find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    include(FetchContent)
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
    set(TC32EMU_YAML_TARGET yaml-cpp::yaml-cpp)
else()
    set(TC32EMU_YAML_TARGET yaml-cpp)
endif()

# Lua 5.4 for scripted parts, built from source so behavior does not depend on
# the system's Lua version. The upstream repository has no CMakeLists.txt.
include(FetchContent)
FetchContent_Declare(lua
    GIT_REPOSITORY https://github.com/lua/lua.git
    GIT_TAG v5.4.8)
FetchContent_MakeAvailable(lua)
file(GLOB TC32EMU_LUA_SOURCES "${lua_SOURCE_DIR}/*.c")
list(FILTER TC32EMU_LUA_SOURCES EXCLUDE REGEX "/(lua|luac|onelua|ltests)\\.c$")
add_library(lua54 STATIC ${TC32EMU_LUA_SOURCES})
target_include_directories(lua54 PUBLIC "${lua_SOURCE_DIR}")
set_target_properties(lua54 PROPERTIES C_STANDARD 99)
if(UNIX)
    target_compile_definitions(lua54 PRIVATE LUA_USE_POSIX)
    target_link_libraries(lua54 PUBLIC m)
endif()
target_compile_options(lua54 PRIVATE -w)

# Chip-independent simulation core: scheduler, pins/nets, endpoints, world.
add_library(simcore
    emulator/src/sim/scheduler.cpp
    emulator/src/sim/net.cpp
    emulator/src/sim/endpoint.cpp
    emulator/src/sim/world.cpp
    emulator/src/sim/uart_endpoint.cpp
    emulator/src/sim/rtt_endpoint.cpp
    emulator/src/sim/sink.cpp
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
)
target_include_directories(simcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(simcore PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(simcore PUBLIC tc32emu mcs51emu ${TC32EMU_YAML_TARGET} lua54)

if(TC32EMU_BUILD_TESTS)
    enable_testing()
    add_executable(tc32emu-tests
        tests/test_main.cpp
        tests/tc32/test_cpu.cpp
        tests/machine/test_tlsr8258.cpp
        tests/machine/test_api.cpp
        tests/radio/test_radio.cpp
    )
    target_link_libraries(tc32emu-tests PRIVATE tc32emu)
    add_test(NAME tc32emu-tests COMMAND tc32emu-tests)

    add_executable(mcs51emu-tests
        tests/mcs51/test_ca51f253l3.cpp
    )
    target_link_libraries(mcs51emu-tests PRIVATE mcs51emu)
    add_test(NAME mcs51emu-tests COMMAND mcs51emu-tests)

    add_executable(simcore-tests
        tests/sim/test_sim.cpp
        tests/sim/test_config.cpp
        tests/sim/test_parts.cpp
        tests/sim/test_periph.cpp
        tests/sim/test_lua.cpp
        tests/sim/test_radio_medium.cpp
        tests/sim/test_attach.cpp
    )
    target_link_libraries(simcore-tests PRIVATE simcore)
    target_compile_definitions(simcore-tests PRIVATE
        TC32EMU_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME simcore-tests COMMAND simcore-tests)

    set(TC32EMU_SAMPLE_GW_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/samplegw_tb03f.bin")
    set(TC32EMU_SAMPLE_LIGHT_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/samplelight_tb03f.bin")
    if(EXISTS "${TC32EMU_SAMPLE_GW_FIRMWARE}" AND
       EXISTS "${TC32EMU_SAMPLE_LIGHT_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zigbee_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zigbee_join.py"
                        "$<TARGET_FILE:tc32-emu>"
                        "${TC32EMU_SAMPLE_GW_FIRMWARE}"
                        "${TC32EMU_SAMPLE_LIGHT_FIRMWARE}"
                        join)
            add_test(
                NAME firmware_zigbee_button_light
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zigbee_join.py"
                        "$<TARGET_FILE:tc32-emu>"
                        "${TC32EMU_SAMPLE_GW_FIRMWARE}"
                        "${TC32EMU_SAMPLE_LIGHT_FIRMWARE}"
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

    set(TC32EMU_ZEPHYR_PM_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-pmtest.bin")
    if(EXISTS "${TC32EMU_ZEPHYR_PM_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zephyr_pm_wake
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zephyr_pm_wake.py"
                        "$<TARGET_FILE:tc32-emu>"
                        "${TC32EMU_ZEPHYR_PM_FIRMWARE}")
            set_tests_properties(firmware_zephyr_pm_wake PROPERTIES
                TIMEOUT 180
                LABELS "firmware;power"
            )
        else()
            message(STATUS
                "Skipping firmware_zephyr_pm_wake: Python 3 interpreter not found")
        endif()
    endif()

    set(TC32EMU_VENDOR_SLEEP_1S_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/vendor_sleep_1s.bin")
    if(EXISTS "${TC32EMU_VENDOR_SLEEP_1S_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_vendor_sleep_1s
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_vendor_sleep_1s.py"
                        "$<TARGET_FILE:tc32-emu>"
                        "${TC32EMU_VENDOR_SLEEP_1S_FIRMWARE}")
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
