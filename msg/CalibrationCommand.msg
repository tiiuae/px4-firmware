# CalibrationCommand.msg
# Updated to use Commander worker_thread.hpp Request enum directly

uint64 timestamp

# Calibration type enum (matching Commander worker_thread.hpp Request enum)
uint8 GYRO_CALIBRATION = 0
uint8 MAG_CALIBRATION = 1
uint8 RC_TRIM_CALIBRATION = 2
uint8 ACCEL_CALIBRATION = 3
uint8 LEVEL_CALIBRATION = 4
uint8 ACCEL_CALIBRATION_QUICK = 5
uint8 AIRSPEED_CALIBRATION = 6
uint8 ESC_CALIBRATION = 7
uint8 MAG_CALIBRATION_QUICK = 8
uint8 BARO_CALIBRATION = 9

uint8 calibration_type

# Command type
uint8 START_CALIBRATION = 1
uint8 STOP_CALIBRATION = 2
uint8 HEALTH_QUERY = 3
uint8 command