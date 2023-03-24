
# Build the px4_init object library, part of the entry point logic

add_library(px4_init OBJECT
	px4_init.cpp
)
add_dependencies(px4_init prebuild_targets)

if (CONFIG_BUILD_FLAT)
	target_link_libraries(px4_init INTERFACE px4_layer)
else()
	target_link_libraries(px4_init INTERFACE px4_kernel_layer)
	target_compile_options(px4_init INTERFACE -D__KERNEL__)
endif()

target_link_libraries(px4_init INTERFACE px4_platform px4_work_queue perf uORB)

if (DEFINED PX4_CRYPTO)
	target_link_libraries(px4_init PRIVATE crypto_backend_interface)
endif()
