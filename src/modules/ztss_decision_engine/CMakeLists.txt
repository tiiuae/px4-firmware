px4_add_module(
	MODULE modules__ztss_decision_engine
	MAIN ztss_decision_engine
	COMPILE_FLAGS
	SRCS
		ztss_decision_engine.cpp
	MODULE_CONFIG
		module.yaml
	# INCLUDE_DIRS
	# 	${CMAKE_CURRENT_LIST_DIR}/..
	# DEPENDS
)
