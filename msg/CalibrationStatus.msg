# CalibrationStatus.msg
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

# Calibration status enums
uint8 CAL_STATUS_PLAIN_MESSAGE                      = 0

# VEHICLE_COMMAND_ACK messages
uint8 CAL_STATUS_COMMAND_ACK_ACCEPTED               = 1
uint8 CAL_STATUS_COMMAND_ACK_TEMPORARILY_REJECTED   = 2
uint8 CAL_STATUS_COMMAND_ACK_DENIED                 = 3
uint8 CAL_STATUS_COMMAND_ACK_UNSUPPORTED            = 4
uint8 CAL_STATUS_COMMAND_ACK_FAILED                 = 5
uint8 CAL_STATUS_COMMAND_ACK_IN_PROGRESS            = 6

# Status messages
uint8 CAL_STATUS_STARTED                            = 8
uint8 CAL_STATUS_DONE                               = 9
uint8 CAL_STATUS_CANCELLED                          = 11 # calibration cancelled by user
uint8 CAL_STATUS_PROGRESS                           = 12

# Status messages with 'orientation'
uint8 CAL_STATUS_ORIENTATION_DETECTED_MSG           = 16
uint8 CAL_STATUS_ORIENTATION_HOLD                   = 17
# uint8 CAL_STATUS_ORIENTATION_SIDE_PROGRESS          = 18 # UNUSED - no logic in Calibration-worker
uint8 CAL_STATUS_ORIENTATION_SIDE_DONE              = 19
uint8 CAL_STATUS_INVALID_AXIS                       = 20 # logged but no state change
uint8 CAL_STATUS_SIDE_ALREADY_COMPLETED             = 21 # side already completed
uint8 CAL_STATUS_ROTATE_VEHICLE                     = 22 # rotate vehicle

# Status messages with 'orientation' and 'result'
uint8 CAL_STATUS_SIDE_RESULT                        = 24 # logged but no state change


uint8 CAL_STATUS_MOTION_RETRY                       = 32 # motion detected during calibration, retrying

# Radio calibration
# uint8 CAL_STATUS_DISABLE_RC                         = 33
# uint8 CAL_STATUS_RESTORE_RC                         = 34


# Error messages (bit 7 set = 128+)
uint8 CAL_STATUS_FAILED                             = 128 # calibration failed:
uint8 CAL_STATUS_ERROR_SENSOR                       = 129 # calibration failed: reading sensor
uint8 CAL_STATUS_ERROR_RESET_CAL                    = 130 # calibration failed: to reset
uint8 CAL_STATUS_ERROR_READ_CAL                     = 131 # calibration failed: to read calibration
uint8 CAL_STATUS_ERROR_APPLY_CAL                    = 132 # calibration failed: to apply calibration
uint8 CAL_STATUS_ERROR_SET_PARAMS                   = 133 # calibration failed: to set parameters

# Error messages without [cal] tag - Critical failures that end calibration
uint8 CAL_STATUS_ERROR_SAVE                         = 160 # calibration save failed
uint8 CAL_STATUS_ERROR_ATT_EST                      = 161 # attitude estimator not running - check system boot  
uint8 CAL_STATUS_ERROR_MOTION                       = 162 # motion during calibration
uint8 CAL_STATUS_ERROR_ROLL                         = 164 # excess roll angle
uint8 CAL_STATUS_ERROR_PITCH                        = 165 # excess pitch angle
uint8 CAL_STATUS_ERROR_REQUIRES_ROTATION            = 166 # Failed: This calibration requires rotation.
uint8 CAL_STATUS_ESC_DENIED                         = 169 # ESC calibration denied - safety button required

uint8 status


uint8 ORIENTATION_NONE  = 0
uint8 ORIENTATION_BACK  = 1
uint8 ORIENTATION_FRONT = 2
uint8 ORIENTATION_LEFT  = 3
uint8 ORIENTATION_RIGHT = 4
uint8 ORIENTATION_UP    = 5
uint8 ORIENTATION_DOWN  = 6
uint8 ORIENTATION_X     = 128
uint8 ORIENTATION_Y     = 129
uint8 ORIENTATION_Z     = 130
uint8 ORIENTATION_ERROR = 255

uint8 orientation

uint8 value

float32 result1
float32 result2
float32 result3

char[128] message
