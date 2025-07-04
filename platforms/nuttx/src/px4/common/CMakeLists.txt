############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

# skip for px4_layer support on an IO board
if(NOT PX4_BOARD MATCHES "io-v2")
	# Kernel side & nuttx flat build common sources
	set(KERNEL_SRCS
		board_crashdump.c
		board_dma_alloc.c
		board_fat_dma_alloc.c
		console_buffer.cpp
		cpuload.cpp
		gpio.c
		print_load.cpp
		tasks.cpp
		px4_atomic.cpp
		px4_nuttx_impl.cpp
		px4_init.cpp
		px4_manifest.cpp
		px4_mtd.cpp
		px4_24xxxx_mtd.c
		px4_crypto.cpp
		progmem_dump.c
	)

	# Kernel side & nuttx flat build common libraries
	set(KERNEL_LIBS
		arch_board_reset
		arch_board_critmon
		arch_version
		nuttx_sched
		crc
	)

if (NOT DEFINED CONFIG_BUILD_FLAT AND "${PX4_PLATFORM}" MATCHES "nuttx")
	# Build the NuttX user and kernel space px4 layers
	include(${CMAKE_CURRENT_SOURCE_DIR}/px4_protected_layers.cmake)

else()
	# Build the flat build px4_layer
	include(${CMAKE_CURRENT_SOURCE_DIR}/px4_layer.cmake)
endif()

else()
	# Build the px4 layer for io_v2
	add_library(px4_layer ${PX4_SOURCE_DIR}/platforms/common/empty.c)
endif()

add_dependencies(px4_layer prebuild_targets)

add_subdirectory(gpio)
add_subdirectory(srgbled)

# Build px4_random
if (DEFINED PX4_CRYPTO)
	add_library(px4_random nuttx_random.c)
	target_link_libraries(px4_random PRIVATE nuttx_crypto)
endif()