# Encodes the redundancy state of the vehicle, as it appears to this FC

uint64 timestamp # time since system start (microseconds)

uint8 fc_number # Number of this FC, 1-4, matches MAV_COMP_ID
uint8 fc_in_act_control # Which FC is in actuator control, 0 - unknown

uint8 UNKNOWN_IN_CONTROL = 0
uint8 FC1_IN_CONTROL = 1
uint8 FC2_IN_CONTROL = 2
uint8 FC3_IN_CONTROL = 3
uint8 FC4_IN_CONTROL = 4

uint32 error_flags # Bitmask of redundancy related errors

# Bit positions for error_flags

uint8 FC1_DISARMED_IN_FLIGHT = 0
uint8 FC2_DISARMED_IN_FLIGHT = 1
uint8 FC3_DISARMED_IN_FLIGHT = 2
uint8 FC4_DISARMED_IN_FLIGHT = 3
uint8 FC1_COMMS_ERR = 4
uint8 FC2_COMMS_ERR = 5
uint8 FC3_COMMS_ERR = 6
uint8 FC4_COMMS_ERR = 7
