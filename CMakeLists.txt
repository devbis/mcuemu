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
    emulator/src/radio.cpp
    emulator/src/api.cpp
)
target_include_directories(tc32emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(tc32emu PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(tc32emu PUBLIC unicorn)

add_executable(tc32-emu emulator/src/main.cpp)
target_link_libraries(tc32-emu PRIVATE tc32emu)

add_library(mcs51emu
    emulator/src/mcs51/ca51f253l3.cpp
)
target_include_directories(mcs51emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(mcs51emu PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mcs51emu PUBLIC unicorn)

add_executable(mcs51-emu emulator/src/mcs51/main.cpp)
target_link_libraries(mcs51-emu PRIVATE mcs51emu)

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
