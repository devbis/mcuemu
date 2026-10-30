# Builds one of the Telink Zigbee SDK sample applications for the TLSR8258 with
# the tc32 GCC toolchain, without the SDK's Eclipse project. The firmware tests
# use the result as the Zigbee coordinator (sampleGW) and as a router (sampleLight).
#
#   cmake -S tests/ci/telink -B build-gw -DAPP=sampleGW \
#         -DSDK_PREFIX=<telink_zigbee_sdk>/tl_zigbee_sdk -DTOOLCHAIN_PREFIX=<tc32>
#   cmake --build build-gw --target sampleGW.bin
#
# Compiler flags, defines and libraries follow the SDK's sampleGW_8258 and
# sampleLight_8258 configurations.
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

if(NOT APP)
    message(FATAL_ERROR "APP must be sampleGW or sampleLight")
endif()
SET(CMAKE_BUILD_TYPE Release)
include(${CMAKE_CURRENT_LIST_DIR}/TelinkSDK.cmake)

PROJECT(${APP} C)
ENABLE_LANGUAGE(ASM)

include(${CMAKE_CURRENT_LIST_DIR}/sdk_sources.cmake)

if(APP STREQUAL "sampleGW")
    ADD_DEFINITIONS(-DCOORDINATOR=1 -D__PROJECT_TL_GW__=1)
    SET(ZB_LIB zb_coordinator)
elseif(APP STREQUAL "sampleLight")
    ADD_DEFINITIONS(-DROUTER=1 -D__PROJECT_TL_DIMMABLE_LIGHT__=1)
    SET(ZB_LIB zb_router)
else()
    message(FATAL_ERROR "unsupported APP ${APP}")
endif()
ADD_DEFINITIONS(-DMCU_CORE_${TELINK_PLATFORM}=1 -DMCU_STARTUP_${TELINK_PLATFORM}=1)

# The samples default to the dongle board; the tests use the EVK v1.2 layout.
# The application is copied so the SDK checkout stays untouched.
SET(APP_DIR ${CMAKE_BINARY_DIR}/app)
file(COPY ${SDK_PREFIX}/apps/${APP}/ DESTINATION ${APP_DIR})
file(READ ${APP_DIR}/app_cfg.h APP_CFG)
string(REPLACE "BOARD_8258_DONGLE//BOARD_8258_EVK_V1P2" "BOARD_8258_EVK_V1P2" APP_CFG "${APP_CFG}")
file(WRITE ${APP_DIR}/app_cfg.h "${APP_CFG}")

# Optional: the channel the coordinator forms its network on (the sample's default is 15).
if(DEFAULT_CHANNEL AND EXISTS ${APP_DIR}/stack_cfg.h)
    file(READ ${APP_DIR}/stack_cfg.h STACK_CFG)
    string(REGEX REPLACE "#define DEFAULT_CHANNEL +[0-9]+" "#define DEFAULT_CHANNEL ${DEFAULT_CHANNEL}" STACK_CFG "${STACK_CFG}")
    file(WRITE ${APP_DIR}/stack_cfg.h "${STACK_CFG}")
endif()

INCLUDE_DIRECTORIES(
    ${APP_DIR}
    ${SDK_PREFIX}/apps/common
    ${SDK_PREFIX}/zigbee/af
    ${SDK_PREFIX}/zigbee/gp
    ${SDK_PREFIX}/zigbee/bdb/includes
    ${SDK_PREFIX}/zigbee/ota
    ${SDK_PREFIX}/zigbee/zcl
    ${SDK_PREFIX}/zigbee/common/includes
    ${SDK_PREFIX}/zigbee/zbapi
    ${SDK_PREFIX}/zbhci
    ${SDK_PREFIX}/proj
    ${SDK_PREFIX}/proj/common
    ${SDK_PREFIX}/platform
)
LINK_DIRECTORIES(${SDK_PREFIX}/zigbee/lib/tc32 ${SDK_PREFIX}/platform/lib)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${SDK_PREFIX}/platform/boot/${TELINK_PLATFORM}/boot_${TELINK_PLATFORM}.link")

file(GLOB APP_SOURCES ${APP_DIR}/*.c ${SDK_PREFIX}/apps/common/*.c)
ADD_EXECUTABLE(${APP} ${APP_SOURCES} ${ZIGBEE_SRC})
TARGET_LINK_LIBRARIES(${APP} -Wl,--start-group ${ZB_LIB} drivers_${TELINK_PLATFORM} -Wl,--end-group)

ADD_BIN_TARGET(${APP} ${CMAKE_CURRENT_LIST_DIR}/tools)
