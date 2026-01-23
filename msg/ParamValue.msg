# Parameter value response

uint64 timestamp		# time since system start (microseconds)

uint8 source_sysid		# System ID that sent this value
uint8 source_compid		# Component ID that sent this value
uint16 parameter_index		# Parameter index that was set
char[16] parameter_name		# Parameter name
float32 value			# Confirmed value (int values cast as float)
