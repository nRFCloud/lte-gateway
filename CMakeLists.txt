#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(HW_VERSION_MAJOR "3")
set(HW_VERSION_MINOR "0")
set(FW_VERSION_MAJOR "1")
set(FW_VERSION_MINOR "4.0")

set(spm_CONF_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/child_secure_partition_manager.conf)

# enable ability to perform FOTA from NCS 1.2.0 gw to newer
# e.g., west build -p auto -b apricity_gateway_nrf9160ns -d build_dbg -- -DFOTA_OLD_GW=y
if(FOTA_OLD_GW)
				set(HW_VERSION_MAJOR "2")
				set(HW_VERSION_MINOR "0")
				set(PM_STATIC_YML_FILE
					${CMAKE_CURRENT_SOURCE_DIR}/old_gw_pm_static.yml
					)
				message("FOTA for old gw")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lte-ble-gateway)
zephyr_compile_definitions(PROJECT_NAME=${PROJECT_NAME})
zephyr_compile_definitions(_POSIX_C_SOURCE=200809L)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
# NORDIC SDK APP END
zephyr_include_directories(src)

# Include application events and configuration headers
zephyr_library_include_directories(
src/ui
src/watchdog
src/flash
src
)


find_package(Git QUIET REQUIRED)

execute_process(
    COMMAND "${GIT_EXECUTABLE}" describe --always HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE res
    OUTPUT_VARIABLE MYAPP_VERSION
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)

set_property(GLOBAL APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/.git/index")

if ("${MYAPP_VERSION}" MATCHES "^.*-(.*)-g.*$")
    # if HEAD is tagged with a form of major.minor.patch this separates the items and uses them instead
    string(REGEX REPLACE "^([0-9]+)\\.([0-9]+)\\.([0-9]+).*$" "\\1;\\2;\\3" _ver_parts "${MYAPP_VERSION}")
    list(GET _ver_parts 0 FW_VERSION_MAJOR)
    list(GET _ver_parts 1 FW_VERSION_MINOR)
    list(GET _ver_parts 2 FW_VERSION_PATCH)
else ()
    set(FW_VERSION_PATCH ${MYAPP_VERSION})
endif ()

#
# utilize settings to autogenerate a header file to adjust the build inside the C code
#
configure_file (
   "src/config.h.in"
   "${PROJECT_SOURCE_DIR}/src/config.h"
   )

target_sources(app PRIVATE src/ble.c)
target_sources(app PRIVATE src/ble_codec.c)
target_sources(app PRIVATE src/ble_conn_mgr.c)
target_sources(app PRIVATE src/gateway.c)
target_sources_ifdef(CONFIG_SHELL app PRIVATE src/cli.c)
target_sources(app PRIVATE src/dfu/peripheral_dfu.c)

add_subdirectory(src/ui)
#add_subdirectory(src/dfu)
add_subdirectory_ifdef(CONFIG_WATCHDOG src/watchdog)
add_subdirectory(src/flash)
