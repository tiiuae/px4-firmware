| Module | Subscriptions | Publications | External Inputs | Description |
|--------|--------------|--------------|----------------|-------------|
| `airship_att_control` | `manual_control_setpoint, vehicle_status` | `vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `` | No description available |
| `airspeed_selector` | `estimator_selector_status, estimator_status, position_setpoint, vehicle_acceleration, vehicle_air_data, vehicle_attitude, vehicle_land_detected, vehicle_local_position, vehicle_status, vtol_vehicle_status` | `airspeed_validated_s, airspeed_wind_s` | `MAVLink` | No description available |
| `assembly_agent` | `` | `` | `` |  |
| `attitude_estimator_q` | `vehicle_attitude, vehicle_attitude_s, vehicle_gps_position, vehicle_local_position, vehicle_magnetometer, vehicle_mocap_odometry, vehicle_visual_odometry` | `vehicle_attitude_s` | `` | No description available |
| `battery_status` | `` | `` | `` | No description available |
| `calibration_bridge` | `` | `calibration_status_s, health_report_s, vehicle_command_s` | `MAVLink` | No description available |
| `camera_feedback` | `gimbal_device_attitude_status, vehicle_attitude, vehicle_global_position` | `camera_capture_s` | `` | No description available |
| `commander` | `action_request, battery_status_s, config_control_setpoints, config_overrides_request, cpuload, differential_pressure_s, iridiumsbd_status, manual_control_setpoint, mission_result_s, offboard_control_mode_s, power_button_state, register_ext_component_request, sensor_accel_s, sensor_correction, sensor_gyro_s, sensor_mag, sensor_mag_s, system_power, unregister_ext_component, vehicle_attitude, vehicle_attitude_s, vehicle_command, vehicle_command_ack_s, vehicle_command_mode_executor, vehicle_global_position_s, vehicle_gps_position, vehicle_land_detected, vehicle_local_position_s, vehicle_status, vehicle_status_s, vtol_vehicle_status` | `actuator_armed_s, actuator_test_s, config_overrides_s, failure_detector_status_s, led_control, mag_worker_data, power_button_state, register_ext_component_reply_s, tune_control, vehicle_command_ack_s, vehicle_command_s, vehicle_control_mode_s, vehicle_status_s` | `MAVLink` | No description available |
| `control_allocator` | `failure_detector_status, vehicle_control_mode, vehicle_status, vehicle_thrust_setpoint, vehicle_torque_setpoint` | `actuator_motors_s, actuator_servos_s, actuator_servos_trim_s, control_allocator_status_s` | `` | No description available |
| `dataman` | `dataman_request` | `dataman_response_s` | `MAVLink` | No description available |
| `differential_drive` | `manual_control_setpoint, parameter_update, vehicle_control_mode, vehicle_status` | `differential_drive_setpoint_s` | `` | No description available |
| `ekf2` | `airspeed, airspeed_validated, landing_target_pose, sensor_selection, sensors_status_imu, vehicle_air_data, vehicle_command, vehicle_gps_position, vehicle_imu_s, vehicle_land_detected, vehicle_magnetometer, vehicle_magnetometer_s, vehicle_optical_flow, vehicle_status, vehicle_status_s, vehicle_visual_odometry` | `T, ekf2_timestamps_s, estimator_aid_source1d_s, estimator_aid_source2d_s, estimator_aid_source3d_s, estimator_bias3d_s, estimator_bias_s, estimator_gps_status_s, estimator_innovations_s, estimator_selector_status_s, estimator_sensor_bias_s, estimator_states_s, estimator_status_flags_s, estimator_status_s, sensor_selection_s, vehicle_attitude_s, vehicle_global_position_s, vehicle_local_position_s, vehicle_odometry_s, vehicle_optical_flow_vel_s, wind_s, yaw_estimator_status_s` | `MAVLink` | No description available |
| `esc_battery` | `` | `` | `` | No description available |
| `events` | `battery_status_s, cpuload_s, failsafe_flags, failsafe_flags_s, vehicle_command, vehicle_status, vehicle_status_s` | `led_control_s, tune_control_s, vehicle_command_ack_s` | `MAVLink` | No description available |
| `flight_mode_manager` | `takeoff_status, vehicle_attitude_setpoint, vehicle_command, vehicle_control_mode_s, vehicle_land_detected_s, vehicle_status_s` | `landing_gear_s, trajectory_setpoint_s, vehicle_constraints_s` | `` | No description available |
| `fw_att_control` | `airspeed_validated_s, autotune_attitude_control_status, manual_control_setpoint, vehicle_angular_velocity, vehicle_attitude_setpoint, vehicle_control_mode, vehicle_land_detected, vehicle_local_position, vehicle_status` | `landing_gear_wheel_s, vehicle_attitude_setpoint_s, vehicle_rates_setpoint_s` | `` | No description available |
| `fw_autotune_attitude_control` | `manual_control_setpoint, vehicle_angular_velocity, vehicle_status` | `` | `MAVLink` | No description available |
| `fw_pos_control` | `airspeed_validated, manual_control_setpoint, position_setpoint_triplet, trajectory_setpoint, vehicle_air_data, vehicle_angular_velocity, vehicle_attitude, vehicle_command, vehicle_control_mode, vehicle_global_position, vehicle_land_detected, vehicle_status, wind` | `figure_eight_status_s, landing_gear_s, launch_detection_status_s, normalized_unsigned_setpoint_s, npfg_status_s, orbit_status_s, position_controller_landing_status_s, position_controller_status_s, tecs_status_s, vehicle_attitude_setpoint_s, vehicle_local_position_setpoint_s` | `` | No description available |
| `fw_rate_control` | `airspeed_validated_s, battery_status, manual_control_setpoint, vehicle_angular_velocity, vehicle_control_mode, vehicle_land_detected, vehicle_rates_setpoint, vehicle_status` | `actuator_controls_status_s, normalized_unsigned_setpoint_s, rate_ctrl_status_s, vehicle_rates_setpoint_s, vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `` | No description available |
| `gimbal` | `gimbal_device_attitude_status, gimbal_device_information, gimbal_manager_set_attitude, gimbal_manager_set_manual_control, manual_control_setpoint, position_setpoint_triplet, vehicle_attitude, vehicle_command, vehicle_global_position, vehicle_land_detected, vehicle_roi` | `gimbal_device_set_attitude_s, gimbal_manager_information_s, gimbal_manager_status_s, mount_orientation_s, vehicle_command_ack_s, vehicle_command_s` | `MAVLink, RC` | No description available |
| `gyro_calibration` | `` | `` | `` | No description available |
| `gyro_fft` | `sensor_gyro_fifo_s, sensor_gyro_s, sensor_selection, vehicle_imu_status` | `sensor_gyro_fft_s` | `` | No description available |
| `land_detector` | `actuator_armed, airspeed_validated, hover_thrust_estimate, launch_detection_status, sensor_selection, takeoff_status, trajectory_setpoint, vehicle_acceleration, vehicle_angular_velocity, vehicle_control_mode, vehicle_imu_status, vehicle_status, vehicle_thrust_setpoint` | `vehicle_land_detected_s` | `` | No description available |
| `landing_target_estimator` | `irlock_report, vehicle_acceleration, vehicle_attitude, vehicle_local_position` | `landing_target_innovations_s, landing_target_pose_s` | `` | No description available |
| `load_mon` | `` | `cpuload_s, task_stack_info_s` | `` | No description available |
| `local_position_estimator` | `actuator_armed_s, distance_sensor_s, landing_target_pose_s, sensor_gps_s, vehicle_air_data_s, vehicle_angular_velocity_s, vehicle_attitude_s, vehicle_command, vehicle_land_detected_s, vehicle_local_position_s, vehicle_odometry_s, vehicle_optical_flow_s` | `` | `MAVLink` | No description available |
| `logger` | `battery_status, manual_control_setpoint, ulog_stream_ack, vehicle_command, vehicle_gps_position, vehicle_status` | `logger_status_s, ulog_stream_s, vehicle_command_ack_s` | `MAVLink, RC` | No description available |
| `mag_bias_estimator` | `parameter_update, vehicle_angular_velocity, vehicle_status` | `magnetometer_bias_estimate_s` | `` | No description available |
| `manual_control` | `action_request_s, manual_control_setpoint_s, vehicle_status` | `action_request_s, landing_gear_s, manual_control_setpoint_s, manual_control_switches_s, vehicle_command_s, vehicle_status_s` | `MAVLink` | No description available |
| `mavlink` | `actuator_armed, autotune_attitude_control_status, event, gimbal_v1_command, home_position, mission_result, mission_s, uavcan_parameter_value, ulog_stream_s, vehicle_attitude, vehicle_command, vehicle_command_ack, vehicle_global_position, vehicle_local_position, vehicle_status` | `actuator_outputs_s, battery_status_s, camera_status_s, cellular_status_s, collision_report_s, debug_array_s, debug_key_value_s, debug_value_s, debug_vect_s, differential_pressure_s, distance_sensor_s, esc_status_s, event, follow_target_s, generator_status_s, gimbal_device_attitude_status_s, gimbal_device_information_s, gimbal_manager_set_attitude_s, gimbal_manager_set_manual_control_s, gps_inject_data_s, input_rc_s, irlock_report_s, landing_target_pose_s, log_message_s, manual_control_setpoint_s, mavlink_tunnel_s, mission_s, obstacle_distance_s, offboard_control_mode_s, onboard_computer_status_s, ping_s, radio_status_s, rc_parameter_map_s, sensor_baro_s, sensor_gps_s, sensor_optical_flow_s, telemetry_status_s, trajectory_setpoint_s, transponder_report_s, tune_control_s, uavcan_parameter_request_s, ulog_stream_ack_s, vehicle_angular_velocity_s, vehicle_attitude_s, vehicle_attitude_setpoint_s, vehicle_command, vehicle_command_ack_s, vehicle_command_s, vehicle_global_position_s, vehicle_local_position_s, vehicle_odometry_s, vehicle_rates_setpoint_s, vehicle_status_s, vehicle_trajectory_bezier_s, vehicle_trajectory_waypoint_s, velocity_limits_s` | `MAVLink, RC` | No description available |
| `mc_att_control` | `autotune_attitude_control_status, manual_control_setpoint, vehicle_attitude_setpoint, vehicle_control_mode, vehicle_land_detected, vehicle_local_position, vehicle_status` | `vehicle_attitude_setpoint_s, vehicle_rates_setpoint_s` | `` | No description available |
| `mc_autotune_attitude_control` | `actuator_controls_status_0, manual_control_setpoint, vehicle_angular_velocity, vehicle_status` | `` | `` | No description available |
| `mc_hover_thrust_estimator` | `vehicle_land_detected, vehicle_local_position_setpoint, vehicle_status` | `hover_thrust_estimate_s` | `` | No description available |
| `mc_pos_control` | `hover_thrust_estimate, trajectory_setpoint, vehicle_constraints, vehicle_control_mode, vehicle_land_detected` | `vehicle_attitude_setpoint_s, vehicle_local_position_setpoint_s` | `MAVLink` | No description available |
| `mc_rate_control` | `battery_status, control_allocator_status, manual_control_setpoint, vehicle_control_mode, vehicle_land_detected, vehicle_rates_setpoint, vehicle_status` | `actuator_controls_status_s, rate_ctrl_status_s, vehicle_rates_setpoint_s, vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `MAVLink` | No description available |
| `moi_agent` | `` | `` | `` |  |
| `muorb` | `` | `` | `` |  |
| `navigator` | `geofence_status_s, home_position, home_position_s, landing_target_pose, mission, mission_s, position_controller_landing_status, position_controller_status_s, transponder_report, vehicle_command, vehicle_global_position, vehicle_global_position_s, vehicle_gps_position, vehicle_land_detected, vehicle_land_detected_s, vehicle_local_position, vehicle_status, vehicle_status_s, wind_s` | `geofence_result_s, geofence_status_s, mission_result_s, mission_s, mode_completed_s, navigator_mission_item_s, position_setpoint_triplet_s, rtl_time_estimate_s, vehicle_command_ack_s, vehicle_command_s, vehicle_roi_s` | `MAVLink` | No description available |
| `payload_deliverer` | `` | `gripper_s, vehicle_command_ack_s, vehicle_command_s` | `MAVLink` | No description available |
| `px4_fw_update_client` | `` | `` | `MAVLink` | No description available |
| `px4iofirmware` | `` | `` | `RC` | No description available |
| `rc_update` | `manual_control_switches_s, rc_parameter_map` | `manual_control_setpoint_s, manual_control_switches_s, rc_channels_s` | `MAVLink, RC` | No description available |
| `redundancy` | `actuator_outputs, esc_status, vehicle_attitude, vehicle_control_mode, vehicle_land_detected, vehicle_status` | `offboard_control_mode_s, redundancy_status_s, vehicle_attitude_setpoint_s, vehicle_command_s` | `MAVLink` | No description available |
| `replay` | `` | `` | `` | No description available |
| `rover_pos_control` | `manual_control_setpoint, position_setpoint_triplet, trajectory_setpoint, vehicle_acceleration_s, vehicle_attitude, vehicle_attitude_setpoint, vehicle_control_mode, vehicle_global_position, vehicle_local_position` | `position_controller_status_s, vehicle_attitude_setpoint_s, vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `MAVLink` | No description available |
| `rust_module_example` | `` | `` | `` |  |
| `secure_udp_proxy` | `` | `` | `` | No description available |
| `sensors` | `adc_report, differential_pressure, sensor_accel, sensor_accel_s, sensor_gyro, sensor_gyro_s, sensor_mag_s, sensor_optical_flow, sensor_selection, vehicle_air_data, vehicle_control_mode, vehicle_imu_s` | `airspeed_s, differential_pressure_s, sensor_combined_s, sensor_selection_s, sensors_status_imu_s` | `MAVLink` | No description available |
| `simulation` | `` | `` | `` |  |
| `temperature_compensation` | `vehicle_command` | `sensor_correction_s, vehicle_command_ack_s, vehicle_command_s` | `MAVLink` | No description available |
| `uuv_att_control` | `manual_control_setpoint, vehicle_angular_velocity, vehicle_attitude_setpoint, vehicle_control_mode, vehicle_rates_setpoint` | `vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `MAVLink` | No description available |
| `uuv_pos_control` | `trajectory_setpoint, vehicle_attitude, vehicle_control_mode` | `vehicle_attitude_setpoint_s` | `MAVLink` | No description available |
| `uxrce_dds_client` | `message_format_request, vehicle_command_ack` | `message_format_response_s, vehicle_command_s` | `MAVLink` | No description available |
| `vtol_att_control` | `action_request, airspeed_validated, fw_virtual_attitude_setpoint, home_position, mc_virtual_attitude_setpoint, position_setpoint_triplet, tecs_status, vehicle_air_data, vehicle_attitude, vehicle_command, vehicle_control_mode, vehicle_land_detected, vehicle_local_position, vehicle_local_position_setpoint, vehicle_status` | `normalized_unsigned_setpoint_s, tiltrotor_extra_controls_s, vehicle_attitude_setpoint_s, vehicle_command_ack_s, vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s, vtol_vehicle_status_s` | `MAVLink` | No description available |
| `zenoh` | `` | `` | `` | No description available |


## Drivers (Physical Inputs)
| Driver | Publishes (uORB) | Physical Input | Description |
|--------|------------------|----------------|-------------|
| `actuators` | `` | `ACTUATORS` |  |
| `adc` | `` | `ADC` |  |
| `barometer` | `` | `BAROMETER` |  |
| `batt_smbus` | `` | `BATT_SMBUS` | No description available |
| `bootloaders` | `` | `BOOTLOADERS` |  |
| `camera_capture` | `camera_trigger_s, vehicle_command_ack_s` | `CAMERA_CAPTURE` | No description available |
| `camera_trigger` | `camera_trigger, vehicle_command_ack_s, vehicle_command_s` | `CAMERA_TRIGGER` | No description available |
| `cdcacm_autostart` | `` | `CDCACM_AUTOSTART` | No description available |
| `cyphal` | `` | `CYPHAL` | No description available |
| `differential_pressure` | `` | `DIFFERENTIAL_PRESSURE` |  |
| `distance_sensor` | `` | `DISTANCE_SENSOR` |  |
| `dshot` | `vehicle_command_ack_s` | `DSHOT` | No description available |
| `gnss` | `` | `GNSS` |  |
| `gpio` | `` | `GPIO` |  |
| `gps` | `gps_dump_s, gps_inject_data_s, satellite_info_s, sensor_gnss_relative_s, sensor_gps_s` | `GPS` | No description available |
| `heater` | `heater_status_s` | `HEATER` | No description available |
| `hygrometer` | `` | `HYGROMETER` |  |
| `imu` | `` | `IMU` |  |
| `imx9_keystore` | `` | `IMX9_KEYSTORE` |  |
| `ins` | `` | `INS` |  |
| `irlock` | `irlock_report_s` | `IRLOCK` | No description available |
| `lights` | `` | `LIGHTS` |  |
| `linux_pwm_out` | `` | `LINUX_PWM_OUT` | No description available |
| `magnetometer` | `` | `MAGNETOMETER` |  |
| `optical_flow` | `` | `OPTICAL_FLOW` |  |
| `osd` | `` | `OSD` |  |
| `pca9685_pwm_out` | `` | `PCA9685_PWM_OUT` | No description available |
| `pfsoc_keystore` | `` | `PFSOC_KEYSTORE` |  |
| `power_monitor` | `` | `POWER_MONITOR` |  |
| `pps_capture` | `pps_capture_s` | `PPS_CAPTURE` | No description available |
| `pwm_esc` | `actuator_outputs_s` | `PWM_ESC` | No description available |
| `pwm_input` | `` | `PWM_INPUT` | No description available |
| `pwm_out` | `` | `PWM_OUT` | No description available |
| `px4io` | `input_rc_s, px4io_status_s, vehicle_command_ack_s` | `PX4IO` | No description available |
| `qshell` | `` | `QSHELL` |  |
| `rc` | `` | `RC` |  |
| `rc_input` | `input_rc_s, vehicle_command_ack_s, vehicle_command_s` | `RC_INPUT` | No description available |
| `roboclaw` | `wheel_encoders_s` | `ROBOCLAW` | No description available |
| `rover_interface` | `rover_status` | `ROVER_INTERFACE` | No description available |
| `rpi_rc_in` | `input_rc_s` | `RPI_RC_IN` | No description available |
| `rpm` | `` | `RPM` |  |
| `safety_button` | `` | `SAFETY_BUTTON` | No description available |
| `smart_battery` | `` | `SMART_BATTERY` |  |
| `ssrc_crypto` | `` | `SSRC_CRYPTO` |  |
| `stub_keystore` | `` | `STUB_KEYSTORE` |  |
| `sw_crypto` | `` | `SW_CRYPTO` |  |
| `tap_esc` | `esc_status_s` | `TAP_ESC` | No description available |
| `tattu_can` | `battery_status_s` | `TATTU_CAN` | No description available |
| `telemetry` | `` | `TELEMETRY` |  |
| `test_ppm` | `` | `TEST_PPM` | No description available |
| `tone_alarm` | `tune_control_s` | `TONE_ALARM` | No description available |
| `transponder` | `` | `TRANSPONDER` |  |
| `tvs` | `sensor_tvs_s` | `TVS` | No description available |
| `uavcan` | `can_interface_status, can_interface_status_s, uavcan_parameter_value_s, vehicle_command_ack_s` | `UAVCAN` | No description available |
| `uavcannode` | `` | `UAVCANNODE` | No description available |
| `uwb` | `` | `UWB` |  |
| `voxl2_io` | `input_rc_s` | `VOXL2_IO` | No description available |
| `wind_sensor` | `` | `WIND_SENSOR` |  |