############################################################################
#
#   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
#
############################################################################
px4_add_module(
	MODULE systemcmds__ele
	MAIN ele
	STACK_MAIN 8192
	SRCS
		ele.cpp
	)
