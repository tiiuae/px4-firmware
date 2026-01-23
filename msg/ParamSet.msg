# Parameter set request

uint64 timestamp		# time since system start (microseconds)

uint8 target_sysid		# Target system ID
uint8 target_compid		# Target component ID
uint16 parameter_index		# Parameter index
char[16] parameter_name		# Parameter name (max 16 chars for MAVLink PARAM_SET)
float32 value			# Parameter value (int values cast as float)
uint8 param_type		# MAVLink parameter type (MAV_PARAM_TYPE_REAL32=1, MAV_PARAM_TYPE_INT32=6)
