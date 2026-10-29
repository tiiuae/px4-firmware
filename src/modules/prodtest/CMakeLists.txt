############################################################################
#
#   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__prodtest
	MAIN prodtest
	NO_DAEMON
	SRCS
		prodtest_main.cpp
	DEPENDS
		# imxrt_gpiosetevent() used by the SPI IMU drivers for data-ready interrupts
		arch_io_pins
	)

# In a full image px4_task_spawn_cmd() / px4_get_taskname() (px4_layer) get
# pulled in by the daemon modules. Without them, the references from
# px4_platform and px4_work_queue appear after the last px4_layer occurrence on
# the link line and stay unresolved, so force the symbol in.
target_link_options(modules__prodtest INTERFACE -Wl,--undefined=px4_task_spawn_cmd)
