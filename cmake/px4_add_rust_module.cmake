############################################################################
#
# Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
include(px4_list_make_absolute)

#=============================================================================
#
#	px4_add_rust_module
#
#	This function builds a static library from a module description.
#
#	Usage:
#		px4_add_module(MODULE <string>
#			MAIN <string>
#			[ STACK_MAIN <string> ]
#			[ STACK_MAX <string> ]
#			[ COMPILE_FLAGS <list> ]
#			[ INCLUDES <list> ]
#			[ DEPENDS <string> ]
#			[ SRCS <list> ]
#			[ MODULE_CONFIG <list> ]
#			[ EXTERNAL ]
#			[ DYNAMIC ]
#			)
#
#	Input:
#		MODULE			: unique name of module
#		MAIN			: entry point
#		STACK			: deprecated use stack main instead
#		STACK_MAIN		: size of stack for main function
#		STACK_MAX		: maximum stack size of any frame
#		COMPILE_FLAGS		: compile flags
#		LINK_FLAGS		: link flags
#		SRCS			: source files
#		MODULE_CONFIG		: yaml config file(s)
#		INCLUDES		: include directories
#		DEPENDS			: targets which this module depends on
#		EXTERNAL		: flag to indicate that this module is out-of-tree
#		DYNAMIC			: don't compile into the px4 binary, but build a separate dynamically loadable module (posix)
#		UNITY_BUILD		: merge all source files and build this module as a single compilation unit
#
#	Output:
#		Static library with name matching MODULE.
#		(Or a shared library when DYNAMIC is specified.)
#
#	Example:
#		px4_add_module(MODULE test
#			SRCS
#				file.cpp
#			STACK_MAIN 1024
#			DEPENDS
#				git_nuttx
#			)
#
function(px4_add_rust_module)

	px4_parse_function_args(
		NAME px4_add_rust_module
		ONE_VALUE MODULE MAIN STACK_MAIN STACK_MAX PRIORITY RUST_LIB STUB_SRC
		MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS MODULE_CONFIG RUST_MOD
		OPTIONS EXTERNAL DYNAMIC UNITY_BUILD NO_DAEMON
		REQUIRED MODULE MAIN
		ARGN ${ARGN})

	list(APPEND ADD_MODULE_ARGS MODULE ${MODULE})
	list(APPEND ADD_MODULE_ARGS MAIN ${MAIN})
	if(DEFINED STACK_MAIN)
		list(APPEND ADD_MODULE_ARGS STACK_MAIN ${STACK_MAIN})
	endif()
	if(DEFINED STACK_MAX)
		list(APPEND ADD_MODULE_ARGS STACK_MAX ${STACK_MAX})
	endif()
	if(DEFINED PRIORITY)
		list(APPEND ADD_MODULE_ARGS PRIORITY ${PRIORITY})
	endif()
	if(DEFINED COMPILE_FLAGS)
		list(APPEND ADD_MODULE_ARGS COMPILE_FLAGS ${COMPILE_FLAGS})
	endif()
	if(DEFINED LINK_FLAGS)
		list(APPEND ADD_MODULE_ARGS LINK_FLAGS ${LINK_FLAGS})
	endif()
	if(DEFINED SRCS)
		list(APPEND ADD_MODULE_ARGS SRCS ${SRCS})
	endif()
	if(DEFINED INCLUDES)
		list(APPEND ADD_MODULE_ARGS INCLUDES ${INCLUDES})
	endif()
	if(DEFINED DEPENDS)
		list(APPEND ADD_MODULE_ARGS DEPENDS ${DEPENDS})
	endif()
	if(DEFINED MODULE_CONFIG)
		list(APPEND ADD_MODULE_ARGS MODULE_CONFIG ${MODULE_CONFIG})
	endif()
	if(EXTERNAL)
		list(APPEND ADD_MODULE_ARGS EXTERNAL)
	endif()
	if(DYNAMIC)
		list(APPEND ADD_MODULE_ARGS DYNAMIC)
	endif()
	if(UNITY_BUILD)
		list(APPEND ADD_MODULE_ARGS UNITY_BUILD)
	endif()
	if(NO_DAEMON)
		list(APPEND ADD_MODULE_ARGS NO_DAEMON)
	endif()

	px4_add_module(
		${ADD_MODULE_ARGS}
	)

	# Build stub library with dummy rust_main function
	px4_add_library(${RUST_LIB} ${STUB_SRC})

	# Build Rust code as a library for the px4 module

	foreach(SRC ${RUST_SRCS})
		list(APPEND RUST_SRCS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${SRC})
	endforeach()

	set(RUST_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/target)
	add_custom_target(rust_build ALL
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${RUST_MOD}
		COMMAND cargo build ${CMAKE_RUST_COMPILER_FLAGS} --release --target-dir ${RUST_TARGET_DIR}
		)

	# Before linking (PRE_LINK), replace the original stub library with Rust build output library
	add_custom_command(TARGET ${MODULE} PRE_LINK
		COMMAND cp ${RUST_TARGET_DIR}/${CMAKE_RUST_TARGET}/release/lib${RUST_MOD}.a ${CMAKE_CURRENT_BINARY_DIR}/lib${RUST_LIB}.a
	)

	# Link Rust library to the PX4 module
	target_link_libraries(${MODULE} PRIVATE ${RUST_LIB})
endfunction()
