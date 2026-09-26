############################################################################
#
#   Copyright (c) 2016, 2019, 2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
if("${PX4_BOARD_LABEL}" STREQUAL  "bootloader")
	add_compile_definitions(BOOTLOADER)

	# px4_platform asks for this on every NuttX build, bootloader included.
	add_library(board_bus_info
		bootloader_bus_info.c
	)
	add_dependencies(board_bus_info nuttx_context)

	add_library(drivers_board
		bootloader_main.c
		init.c
		usb.c
		imxrt_flexspi_nor_boot.c
		imxrt_flexspi_nor_flash.c
		imxrt_clockconfig.c
		timer_config.cpp
	)
	target_link_libraries(drivers_board
		PRIVATE
			nuttx_arch # sdio
			nuttx_drivers # sdio
			px4_layer #gpio
			arch_io_pins # iotimer
			arch_board_romapi
			bootloader
	)
	target_include_directories(drivers_board PRIVATE ${PX4_SOURCE_DIR}/platforms/nuttx/src/bootloader/common)

else()
	if(CONFIG_IMXRT1170_FLEXSPI_FRAM)
	    list(APPEND SRCS
			imxrt_flexspi_fram.c
	    )
	endif()

	# px4_platform links board_bus_info unconditionally, so every board that
	# has buses has to offer it. Same split as the saluki boards.
	add_library(board_bus_info
		i2c.cpp
		spi.cpp
	)
	add_dependencies(board_bus_info nuttx_context)

	px4_add_library(drivers_board
		autoleds.c
		automount.c
		#can.c
		init.c
		led.c
		mtd.cpp
		sdhc.c
		timer_config.cpp
		usb.c
		imxrt_flexspi_fram.c
		imxrt_flexspi_nor_boot.c
		imxrt_flexspi_nor_flash.c
		imxrt_clockconfig.c
		${SRCS}
	)

	target_link_libraries(drivers_board
		PRIVATE
			arch_board_hw_info
			arch_board_romapi
			arch_spi
			board_bus_info
			drivers__led # drv_led_start
			nuttx_arch # sdio
			nuttx_drivers # sdio
			px4_layer
	)
endif()
