cmake_minimum_required(VERSION 3.20)
project(tc32_emu VERSION 0.1.0 LANGUAGES C CXX)

option(TC32EMU_BUILD_TESTS "Build the emulator self-tests" ON)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/engine/unicorn/CMakeLists.txt")
    message(FATAL_ERROR
        "Unicorn source checkout is missing. Run scripts/bootstrap-unicorn.sh first.")
endif()

# The board wrapper depends on the TC32 architecture implemented in this local
# Unicorn checkout. Keep the engine as a separate Git repository under ./engine.
set(UNICORN_ARCH "tc32" CACHE STRING "Architectures built for the TLSR8258 wrapper" FORCE)
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
endif()
