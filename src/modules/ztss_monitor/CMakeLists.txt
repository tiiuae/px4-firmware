# add_subdirectory(ztss_cases/ztss_actuator_failure)
add_subdirectory(ztss_cases/ztss_attitude_failure)
# add_subdirectory(ztss_cases/ztss_case_nan_control_values)
add_subdirectory(ztss_cases/ztss_ekf_divergence)
add_subdirectory(ztss_cases/ztss_gps_failure)
add_subdirectory(ztss_cases/ztss_imu_failure)
# add_subdirectory(ztss_cases/ztss_invalid_inputs_controller)
add_subdirectory(ztss_cases/ztss_magnetometer_failure)
add_subdirectory(ztss_cases/ztss_barometer_failure)
add_subdirectory(ztss_cases/ztss_motor_esc_failure)
add_subdirectory(ztss_cases/ztss_motor_saturation)
add_subdirectory(ztss_cases/ztss_pose_estimate_lost)
add_subdirectory(ztss_cases/ztss_vibration)

px4_add_module(
	MODULE modules__ztss_monitor
	MAIN ztss_monitor
	COMPILE_FLAGS
	SRCS
		ztss_monitor.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		# ztss_actuator_failure
		ztss_case_attitude_failure
		ztss_ekf_divergence
		ztss_gps_failure
		ztss_imu_failure
		ztss_magnetometer_failure
		ztss_barometer_failure
		# ztss_invalid_inputs
		ztss_motor_esc_failure
		ztss_motor_saturation
		ztss_pose_estimate_lost
		ztss_vibration
)

# px4_add_unit_gtest(SRC mag_calibration_test.cpp LINKLIBS modules__commander)
# px4_add_functional_gtest(SRC ModeManagementTest.cpp LINKLIBS modules__commander)
