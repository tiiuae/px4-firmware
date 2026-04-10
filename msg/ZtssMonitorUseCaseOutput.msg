uint64 timestamp

uint8 INFO = 0
uint8 WARN = 1
uint8 CRITICAL = 2

uint8 MARGIN_LOW=3
uint8 MARGIN_HIGH=4

bool healthy
uint8 severity
uint8 margin # e.g. innovation margin, sat count margin


# TOPICS ztss_use_case_attitude_failure ztss_use_case_barometer_failure ztss_use_case_ekf_divergence ztss_use_case_gps_failure ztss_use_case_imu_failure ztss_use_case_magnetometer_failure ztss_use_case_motor_esc_failure ztss_use_case_motor_saturation ztss_use_case_pose_estimate_lost ztss_use_case_vibration

