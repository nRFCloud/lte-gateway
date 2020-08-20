#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(spm_CONF_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/child_secure_partition_manager.conf)

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
)

target_sources(app PRIVATE src/ble.c)
target_sources(app PRIVATE src/ble_codec.c)
target_sources(app PRIVATE src/ble_conn_mgr.c)
target_sources(app PRIVATE src/gateway.c)
target_sources(app PRIVATE src/dfu/peripheral_dfu.c)

add_subdirectory(src/ui)
#add_subdirectory(src/dfu)
add_subdirectory_ifdef(CONFIG_WATCHDOG src/watchdog)
