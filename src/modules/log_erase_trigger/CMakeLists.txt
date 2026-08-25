############################################################################
#
#   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__log_erase_trigger
	MAIN log_erase_trigger
	SRCS
		LogEraseTrigger.cpp
		LogEraseTrigger.hpp
	DEPENDS
		log_utils
		px4_work_queue
	)
