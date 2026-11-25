cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sleepselftest)
target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE $ENV{ZEPHYR_BASE}/soc/phyplus/phy62x2)

# -DNO_RC=1 builds the variant that clears the RC32K wake-up bit before the first sleep: the chip must
# not wake (on the board it never does), which the emulator test checks
if(NO_RC)
  target_compile_definitions(app PRIVATE NO_RC=1)
endif()
