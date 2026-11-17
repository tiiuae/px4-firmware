############################################################################
#
#   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__keystore_zeroize_trigger
	MAIN keystore_zeroize_trigger
	SRCS
		KeystoreZeroizeTrigger.cpp
		KeystoreZeroizeTrigger.hpp
	DEPENDS
		keystore_backend
		px4_work_queue
	)
