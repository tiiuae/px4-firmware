add_subdirectory(safe_actions)


px4_add_module(
	MODULE modules__ztss_safe_actions
	MAIN ztss_safe_actions
	COMPILE_FLAGS
	SRCS
		ztss_safe_actions.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		ztss_action_base
		ztss_safe_hold
		ztss_safe_none
		ztss_safe_land
		ztss_parachute
)
