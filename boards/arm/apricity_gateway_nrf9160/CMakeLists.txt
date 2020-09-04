# Kconfig - APRICITY GATEWAY NRF9160 board configuration
#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic

if(CONFIG_BOARD_APRICITY_GATEWAY_NRF9160 AND NOT DEFINED CONFIG_MCUBOOT)
	zephyr_library()
endif()

if(CONFIG_BOARD_APRICITY_GATEWAY_NRF9160NS)
	zephyr_library()
endif()

zephyr_library_sources(nrf52840_reset.c)
