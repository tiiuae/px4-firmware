# PX4 uORB Graph

## **Drivers → uORB Publications**

| Driver | Physical Input | Publishes | Path |
|--------|----------------|-----------|------|
| `actuators` | `actuators` | `actuator_outputs_s, esc_status_s` | `src/drivers/actuators` |
| `adc` | `adc` | `adc_report_s, system_power_s` | `src/drivers/adc` |
| `barometer` | `Barometer` | `sensor_baro_s` | `src/drivers/barometer` |
| `batt_smbus` | `batt_smbus` | `ORB_ID` | `src/drivers/batt_smbus` |
| `bootloaders` | `bootloaders` | `` | `src/drivers/bootloaders` |
| `camera_capture` | `camera_capture` | `camera_trigger_s, vehicle_command_ack_s` | `src/drivers/camera_capture` |
| `camera_trigger` | `camera_trigger` | `ORB_ID, vehicle_command_ack_s, vehicle_command_s` | `src/drivers/camera_trigger` |
| `cdcacm_autostart` | `cdcacm_autostart` | `` | `src/drivers/cdcacm_autostart` |
| `cyphal` | `cyphal` | `T, battery_status_s, output_control_s` | `src/drivers/cyphal` |
| `differential_pressure` | `differential_pressure` | `differential_pressure_s` | `src/drivers/differential_pressure` |
| `distance_sensor` | `distance_sensor` | `ORB_ID, obstacle_distance_s` | `src/drivers/distance_sensor` |
| `dshot` | `dshot` | `vehicle_command_ack_s` | `src/drivers/dshot` |
| `gnss` | `gnss` | `gps_dump_s, gps_inject_data_s, satellite_info_s, sensor_gps_s` | `src/drivers/gnss` |
| `gpio` | `gpio` | `gpio_in_s` | `src/drivers/gpio` |
| `gps` | `GPS` | `gps_dump_s, gps_inject_data_s, satellite_info_s, sensor_gnss_relative_s, sensor_gps_s` | `src/drivers/gps` |
| `heater` | `heater` | `heater_status_s` | `src/drivers/heater` |
| `hygrometer` | `hygrometer` | `sensor_hygrometer_s` | `src/drivers/hygrometer` |
| `imu` | `IMU` | `sensor_baro_s` | `src/drivers/imu` |
| `imx9_keystore` | `imx9_keystore` | `` | `src/drivers/imx9_keystore` |
| `ins` | `ins` | `estimator_status_s, sensor_baro_s, sensor_gps_s, sensor_selection_s, vehicle_attitude_s, vehicle_global_position_s, vehicle_local_position_s` | `src/drivers/ins` |
| `irlock` | `irlock` | `irlock_report_s` | `src/drivers/irlock` |
| `lights` | `lights` | `` | `src/drivers/lights` |
| `linux_pwm_out` | `linux_pwm_out` | `` | `src/drivers/linux_pwm_out` |
| `magnetometer` | `Magnetometer` | `` | `src/drivers/magnetometer` |
| `optical_flow` | `optical_flow` | `sensor_optical_flow_s` | `src/drivers/optical_flow` |
| `osd` | `osd` | `` | `src/drivers/osd` |
| `pca9685_pwm_out` | `pca9685_pwm_out` | `` | `src/drivers/pca9685_pwm_out` |
| `pfsoc_keystore` | `pfsoc_keystore` | `` | `src/drivers/pfsoc_keystore` |
| `power_monitor` | `power_monitor` | `power_monitor_s` | `src/drivers/power_monitor` |
| `pps_capture` | `pps_capture` | `pps_capture_s` | `src/drivers/pps_capture` |
| `pwm_esc` | `pwm_esc` | `actuator_outputs_s` | `src/drivers/pwm_esc` |
| `pwm_input` | `pwm_input` | `` | `src/drivers/pwm_input` |
| `pwm_out` | `pwm_out` | `` | `src/drivers/pwm_out` |
| `px4io` | `px4io` | `input_rc_s, px4io_status_s, vehicle_command_ack_s` | `src/drivers/px4io` |
| `qshell` | `qshell` | `qshell_req_s, qshell_retval_s` | `src/drivers/qshell` |
| `rc` | `rc` | `input_rc_s` | `src/drivers/rc` |
| `rc_input` | `rc_input` | `input_rc_s, vehicle_command_ack_s, vehicle_command_s` | `src/drivers/rc_input` |
| `roboclaw` | `roboclaw` | `wheel_encoders_s` | `src/drivers/roboclaw` |
| `rover_interface` | `rover_interface` | `ORB_ID` | `src/drivers/rover_interface` |
| `rpi_rc_in` | `rpi_rc_in` | `input_rc_s` | `src/drivers/rpi_rc_in` |
| `rpm` | `rpm` | `rpm_s` | `src/drivers/rpm` |
| `safety_button` | `safety_button` | `` | `src/drivers/safety_button` |
| `smart_battery` | `smart_battery` | `ORB_ID` | `src/drivers/smart_battery` |
| `ssrc_crypto` | `ssrc_crypto` | `` | `src/drivers/ssrc_crypto` |
| `stub_keystore` | `stub_keystore` | `` | `src/drivers/stub_keystore` |
| `sw_crypto` | `sw_crypto` | `` | `src/drivers/sw_crypto` |
| `tap_esc` | `tap_esc` | `esc_status_s` | `src/drivers/tap_esc` |
| `tattu_can` | `tattu_can` | `battery_status_s` | `src/drivers/tattu_can` |
| `telemetry` | `telemetry` | `ORB_ID, iridiumsbd_status_s` | `src/drivers/telemetry` |
| `test_ppm` | `test_ppm` | `` | `src/drivers/test_ppm` |
| `tone_alarm` | `tone_alarm` | `tune_control_s` | `src/drivers/tone_alarm` |
| `transponder` | `transponder` | `transponder_report_s` | `src/drivers/transponder` |
| `tvs` | `tvs` | `sensor_tvs_s` | `src/drivers/tvs` |
| `uavcan` | `uavcan` | `ORB_ID, _orb_topic, can_interface_status_s, esc_status_s, sensor_baro_s, uavcan_parameter_value_s, vehicle_command_ack_s` | `src/drivers/uavcan` |
| `uavcannode` | `uavcannode` | `actuator_armed_s, actuator_motors_s, actuator_servos_s, gps_inject_data_s, led_control_s, tune_control_s` | `src/drivers/uavcannode` |
| `uwb` | `uwb` | `sensor_uwb_s` | `src/drivers/uwb` |
| `voxl2_io` | `voxl2_io` | `input_rc_s` | `src/drivers/voxl2_io` |
| `wind_sensor` | `wind_sensor` | `sensor_airflow_s` | `src/drivers/wind_sensor` |


## **Modules → Subscriptions / Publications / Driver Inputs**

| Module | Subscribes | Publishes | External Inputs | Path |
|--------|------------|-----------|-----------------|------|
| `airship_att_control` | `` | `vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `actuators, imu, ins, rc` | `src/modules/airship_att_control` |
| `airspeed_selector` | `` | `airspeed_validated_s, airspeed_wind_s` | `gps, imu, ins, rc` | `src/modules/airspeed_selector` |
| `assembly_agent` | `` | `` | `` | `src/modules/assembly_agent` |
| `attitude_estimator_q` | `vehicle_attitude_s` | `vehicle_attitude_s` | `gps, imu, ins, magnetometer, rc` | `src/modules/attitude_estimator_q` |
| `battery_status` | `` | `` | `adc, ins, rc` | `src/modules/battery_status` |
| `calibration_bridge` | `` | `calibration_status_s, health_report_s, vehicle_command_s` | `ins, magnetometer, rc` | `src/modules/calibration_bridge` |
| `camera_feedback` | `` | `camera_capture_s` | `camera_capture, camera_trigger, ins, rc` | `src/modules/camera_feedback` |
| `commander` | `battery_status_s, differential_pressure_s, mission_result_s, offboard_control_mode_s, sensor_accel_s, sensor_gyro_s, sensor_mag_s, vehicle_attitude_s, vehicle_command_ack_s, vehicle_global_position_s, vehicle_local_position_s, vehicle_status_s` | `ORB_ID, actuator_armed_s, actuator_test_s, arming_check_request_s, config_overrides_s, failsafe_flags_s, failure_detector_status_s, health_report_s, register_ext_component_reply_s, vehicle_command_ack_s, vehicle_command_s, vehicle_control_mode_s, vehicle_status_s` | `actuators, adc, barometer, differential_pressure, distance_sensor, dshot, gnss, gps, imu, ins, magnetometer, optical_flow, pwm_input, pwm_out, rc, rc_input, rpm, safety_button, telemetry, tone_alarm, uavcan` | `src/modules/commander` |
| `control_allocator` | `` | `actuator_motors_s, actuator_servos_s, actuator_servos_trim_s, control_allocator_status_s` | `actuators, adc, imu, ins, rc` | `src/modules/control_allocator` |
| `dataman` | `` | `dataman_response_s` | `imu, ins, rc` | `src/modules/dataman` |
| `differential_drive` | `` | `actuator_motors_s, differential_drive_setpoint_s` | `imu, ins, rc` | `src/modules/differential_drive` |
| `ekf2` | `vehicle_imu_s, vehicle_magnetometer_s, vehicle_status_s` | `T, ekf2_timestamps_s, estimator_aid_source1d_s, estimator_aid_source2d_s, estimator_aid_source3d_s, estimator_bias3d_s, estimator_bias_s, estimator_gps_status_s, estimator_innovations_s, estimator_selector_status_s, estimator_sensor_bias_s, estimator_states_s, estimator_status_flags_s, estimator_status_s, sensor_selection_s, vehicle_attitude_s, vehicle_global_position_s, vehicle_local_position_s, vehicle_odometry_s, vehicle_optical_flow_vel_s, wind_s, yaw_estimator_status_s` | `barometer, distance_sensor, gnss, gps, imu, ins, magnetometer, optical_flow, osd, rc` | `src/modules/ekf2` |
| `esc_battery` | `` | `` | `ins, rc` | `src/modules/esc_battery` |
| `events` | `battery_status_s, cpuload_s, failsafe_flags_s, vehicle_status_s` | `led_control_s, tune_control_s, vehicle_command_ack_s` | `gps, rc` | `src/modules/events` |
| `flight_mode_manager` | `home_position_s, position_setpoint_triplet_s, vehicle_control_mode_s, vehicle_land_detected_s, vehicle_local_position_s, vehicle_status_s` | `follow_target_estimator_s, follow_target_status_s, gimbal_manager_set_attitude_s, landing_gear_s, orbit_status_s, trajectory_setpoint_s, vehicle_command_s, vehicle_constraints_s` | `gps, imu, ins, rc, telemetry` | `src/modules/flight_mode_manager` |
| `fw_att_control` | `airspeed_validated_s` | `landing_gear_wheel_s, vehicle_attitude_setpoint_s, vehicle_rates_setpoint_s` | `actuators, imu, ins, rc` | `src/modules/fw_att_control` |
| `fw_autotune_attitude_control` | `` | `` | `imu, ins, rc` | `src/modules/fw_autotune_attitude_control` |
| `fw_pos_control` | `` | `figure_eight_status_s, landing_gear_s, launch_detection_status_s, normalized_unsigned_setpoint_s, npfg_status_s, orbit_status_s, position_controller_landing_status_s, position_controller_status_s, tecs_status_s, vehicle_attitude_setpoint_s, vehicle_local_position_setpoint_s` | `actuators, gnss, gps, imu, ins, rc, rpm` | `src/modules/fw_pos_control` |
| `fw_rate_control` | `airspeed_validated_s` | `actuator_controls_status_s, normalized_unsigned_setpoint_s, rate_ctrl_status_s, vehicle_rates_setpoint_s, vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `actuators, imu, ins, rc` | `src/modules/fw_rate_control` |
| `gimbal` | `` | `gimbal_device_set_attitude_s, gimbal_manager_information_s, gimbal_manager_status_s, mount_orientation_s, vehicle_command_ack_s, vehicle_command_s` | `gps, imu, ins, rc` | `src/modules/gimbal` |
| `gyro_calibration` | `` | `` | `imu, ins, rc` | `src/modules/gyro_calibration` |
| `gyro_fft` | `sensor_gyro_fifo_s, sensor_gyro_s` | `sensor_gyro_fft_s` | `adc, imu, ins, rc` | `src/modules/gyro_fft` |
| `land_detector` | `` | `vehicle_land_detected_s` | `gps, imu, ins, rc` | `src/modules/land_detector` |
| `landing_target_estimator` | `` | `landing_target_innovations_s, landing_target_pose_s` | `irlock, rc` | `src/modules/landing_target_estimator` |
| `load_mon` | `` | `cpuload_s, task_stack_info_s` | `rc` | `src/modules/load_mon` |
| `local_position_estimator` | `actuator_armed_s, distance_sensor_s, landing_target_pose_s, sensor_gps_s, vehicle_air_data_s, vehicle_angular_velocity_s, vehicle_attitude_s, vehicle_land_detected_s, vehicle_local_position_s, vehicle_odometry_s, vehicle_optical_flow_s` | `` | `distance_sensor, gps, imu, ins, optical_flow, rc` | `src/modules/local_position_estimator` |
| `logger` | `` | `logger_status_s, ulog_stream_s, vehicle_command_ack_s` | `actuators, adc, camera_capture, camera_trigger, differential_pressure, distance_sensor, gnss, gps, heater, hygrometer, imu, ins, irlock, magnetometer, optical_flow, osd, pps_capture, px4io, rc, rpm, telemetry, transponder` | `src/modules/logger` |
| `mag_bias_estimator` | `` | `magnetometer_bias_estimate_s` | `magnetometer, rc` | `src/modules/mag_bias_estimator` |
| `manual_control` | `action_request_s, manual_control_setpoint_s` | `action_request_s, landing_gear_s, manual_control_setpoint_s, manual_control_switches_s, vehicle_command_s, vehicle_status_s` | `actuators, imu, ins, rc` | `src/modules/manual_control` |
| `mavlink` | `mission_s, ulog_stream_s` | `ORB_ID, actuator_outputs_s, battery_status_s, camera_status_s, cellular_status_s, collision_report_s, debug_array_s, debug_key_value_s, debug_value_s, debug_vect_s, differential_pressure_s, distance_sensor_s, esc_status_s, follow_target_s, generator_status_s, gimbal_device_attitude_status_s, gimbal_device_information_s, gimbal_manager_set_attitude_s, gimbal_manager_set_manual_control_s, gps_inject_data_s, input_rc_s, irlock_report_s, landing_target_pose_s, log_message_s, manual_control_setpoint_s, mavlink_tunnel_s, mission_s, obstacle_distance_s, offboard_control_mode_s, onboard_computer_status_s, ping_s, radio_status_s, rc_parameter_map_s, sensor_baro_s, sensor_gps_s, sensor_optical_flow_s, telemetry_status_s, trajectory_setpoint_s, transponder_report_s, tune_control_s, uavcan_parameter_request_s, ulog_stream_ack_s, vehicle_angular_velocity_s, vehicle_attitude_s, vehicle_attitude_setpoint_s, vehicle_command_ack_s, vehicle_command_s, vehicle_global_position_s, vehicle_local_position_s, vehicle_odometry_s, vehicle_rates_setpoint_s, vehicle_status_s, vehicle_trajectory_bezier_s, vehicle_trajectory_waypoint_s, velocity_limits_s` | `adc, barometer, camera_capture, camera_trigger, differential_pressure, distance_sensor, gps, hygrometer, imu, ins, irlock, magnetometer, optical_flow, osd, pwm_out, rc, rc_input, rpm, telemetry, transponder, uavcan` | `src/modules/mavlink` |
| `mc_att_control` | `` | `vehicle_attitude_setpoint_s, vehicle_rates_setpoint_s` | `imu, ins, rc` | `src/modules/mc_att_control` |
| `mc_autotune_attitude_control` | `` | `` | `imu, ins, rc` | `src/modules/mc_autotune_attitude_control` |
| `mc_hover_thrust_estimator` | `` | `hover_thrust_estimate_s` | `imu, ins, rc` | `src/modules/mc_hover_thrust_estimator` |
| `mc_pos_control` | `goto_setpoint_s` | `trajectory_setpoint_s, vehicle_attitude_setpoint_s, vehicle_constraints_s, vehicle_local_position_setpoint_s` | `imu, ins, rc` | `src/modules/mc_pos_control` |
| `mc_rate_control` | `` | `actuator_controls_status_s, rate_ctrl_status_s, vehicle_rates_setpoint_s, vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `imu, ins, rc` | `src/modules/mc_rate_control` |
| `moi_agent` | `` | `` | `` | `src/modules/moi_agent` |
| `muorb` | `` | `` | `ins, rc` | `src/modules/muorb` |
| `navigator` | `geofence_status_s, home_position_s, mission_s, position_controller_status_s, vehicle_global_position_s, vehicle_land_detected_s, vehicle_status_s, wind_s` | `ORB_ID, geofence_result_s, geofence_status_s, mission_result_s, mission_s, mode_completed_s, navigator_mission_item_s, position_setpoint_triplet_s, rtl_time_estimate_s, vehicle_command_ack_s, vehicle_command_s, vehicle_roi_s` | `adc, camera_trigger, gps, imu, ins, osd, pwm_out, rc, transponder` | `src/modules/navigator` |
| `payload_deliverer` | `` | `gripper_s, vehicle_command_ack_s, vehicle_command_s` | `ins, rc` | `src/modules/payload_deliverer` |
| `px4_fw_update_client` | `` | `` | `adc, imu, ins, rc` | `src/modules/px4_fw_update_client` |
| `px4iofirmware` | `` | `` | `adc, gpio, heater, imu, ins, pwm_out, px4io, rc, rc_input, safety_button, telemetry` | `src/modules/px4iofirmware` |
| `rc_update` | `manual_control_switches_s` | `manual_control_setpoint_s, manual_control_switches_s, rc_channels_s` | `imu, ins, px4io, rc, rc_input` | `src/modules/rc_update` |
| `redundancy` | `` | `offboard_control_mode_s, redundancy_status_s, vehicle_attitude_setpoint_s, vehicle_command_s` | `adc, gpio, ins, rc` | `src/modules/redundancy` |
| `replay` | `` | `sub` | `distance_sensor, gps, ins, magnetometer, optical_flow, rc` | `src/modules/replay` |
| `rover_pos_control` | `vehicle_acceleration_s` | `position_controller_status_s, vehicle_attitude_setpoint_s, vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `actuators, gps, imu, ins, rc, rover_interface` | `src/modules/rover_pos_control` |
| `rust_module_example` | `` | `` | `` | `src/modules/rust_module_example` |
| `secure_udp_proxy` | `` | `` | `ins, rc` | `src/modules/secure_udp_proxy` |
| `sensors` | `sensor_accel_s, sensor_gyro_fifo_s, sensor_gyro_s, sensor_mag_s, vehicle_imu_s, vehicle_imu_status_s` | `airspeed_s, differential_pressure_s, sensor_combined_s, sensor_gps_s, sensor_preflight_mag_s, sensor_selection_s, sensorsData, sensors_status_imu_s, sensors_status_s, vehicle_acceleration_s, vehicle_air_data_s, vehicle_angular_velocity_s, vehicle_command_ack_s, vehicle_imu_s, vehicle_imu_status_s, vehicle_magnetometer_s, vehicle_optical_flow_s, vehicle_optical_flow_vel_s` | `adc, barometer, differential_pressure, distance_sensor, gps, imu, ins, magnetometer, optical_flow, rc, rpm` | `src/modules/sensors` |
| `simulation` | `` | `actuator_outputs_s, airspeed_s, battery_status_s, differential_pressure_s, distance_sensor_s, esc_status_s, input_rc_s, irlock_report_s, rpm_s, sensor_accel_s, sensor_baro_s, sensor_gps_s, sensor_gyro_s, sensor_optical_flow_s, vehicle_angular_velocity_s, vehicle_attitude_s, vehicle_command_ack_s, vehicle_global_position_s, vehicle_local_position_s, vehicle_odometry_s, wheel_encoders_s` | `actuators, adc, barometer, differential_pressure, distance_sensor, gps, imu, ins, irlock, magnetometer, optical_flow, pwm_out, rc, rc_input, rpm, telemetry` | `src/modules/simulation` |
| `temperature_compensation` | `` | `led_control_s, sensor_correction_s, vehicle_command_ack_s, vehicle_command_s` | `barometer, imu, ins, magnetometer, rc` | `src/modules/temperature_compensation` |
| `uuv_att_control` | `` | `vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s` | `actuators, imu, ins, rc` | `src/modules/uuv_att_control` |
| `uuv_pos_control` | `` | `vehicle_attitude_setpoint_s` | `actuators, ins, rc` | `src/modules/uuv_pos_control` |
| `uxrce_dds_client` | `` | `message_format_response_s, vehicle_command_s` | `gps, imu, ins, osd, rc` | `src/modules/uxrce_dds_client` |
| `vtol_att_control` | `` | `normalized_unsigned_setpoint_s, tiltrotor_extra_controls_s, vehicle_attitude_setpoint_s, vehicle_command_ack_s, vehicle_thrust_setpoint_s, vehicle_torque_setpoint_s, vtol_vehicle_status_s` | `actuators, adc, imu, ins, rc` | `src/modules/vtol_att_control` |
| `zenoh` | `` | `_uorb_meta` | `adc, imu, ins, rc` | `src/modules/zenoh` |
