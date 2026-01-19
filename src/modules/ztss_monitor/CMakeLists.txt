# dummy case for ztss monitor test
add_subdirectory(ztss_cases/ztss_case_dummy)

px4_add_module(
	MODULE modules__ztss_monitor
	MAIN ztss_monitor
	COMPILE_FLAGS
	SRCS
		ztss_monitor.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		ztss_case_dummy
)

# px4_add_unit_gtest(SRC mag_calibration_test.cpp LINKLIBS modules__commander)
# px4_add_functional_gtest(SRC ModeManagementTest.cpp LINKLIBS modules__commander)
