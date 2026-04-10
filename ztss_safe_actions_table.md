# Saluki FT ZTSS Framework cases.

## List of all Monitor cases

| #  | Safety Case                    | Module Detecting It             | Trigger Condition                                 | Parameter(s)                                                 | Safe Action                      	| Imp Due Date 	|
| -- | ------------------------------ | ------------------------------- | ------------------------------------------------- | ------------------------------------------------------------ | -------------------------------- 	| -------------	|
| 1  | Attitude failure (flip/tumble) | FailureDetector                 | Roll or pitch exceeds threshold for time window   | `FD_FAIL_R`, `FD_FAIL_P`, `FD_FAIL_R_TTRI`, `FD_FAIL_P_TTRI` | Hold - > Terminate -> Parachute  	|	16-04	|
| 2  | Motor / ESC failure            | FailureDetector + ESC telemetry | ESC telemetry reports error or timeout            | `FD_ACT_EN`                                                  | Hold Attitude		    	|	16-04	|
| 3  | Critical battery               | Commander + Battery module      | Battery voltage or capacity below threshold       | `BAT_LOW_THR`                                                | Safe Land                          |		|
| 4  | Position estimate lost         | Commander + EKF                 | `vehicle_local_position.valid = false`            | none                                                         | Land - Attitude	                |	16-04	|
| 5  | Velocity estimate lost         | EKF + Commander                 | Velocity validity lost                            | none                                                         | Hold - Atitude 	                |	16-04	|
| 6  | GPS failure                    | EKF2                            | Innovation errors / loss of GPS data              | `EKF2_*` parameters                                          | Land - Attitude			|	16-04	|
| 7  | EKF divergence                 | EKF2                            | Innovation test ratio too large                   | `EKF2_*` thresholds                                          | Hold Position	                |	16-04	|
| 8  | Geofence violation             | Navigator                       | Vehicle outside allowed area                      | `GF_ACTION`                                                  | CBF		                |		|
| 9  | Sensor failure                 | Sensors module                  | Sensor missing or unhealthy                       | various                                                      | Hand Over			        |	16-04	|
| 10 | All IMU failure                | Sensors + estimator             | IMU health check fails                            | none                                                         | Hand Over		              	|	v1.0	|
| 11 | Barometer failure              | Sensors                         | Barometer invalid                                 | none                                                         | Hand Over                    	|	v1.0	|
| 12 | Magnetometer failure           | Sensors                         | Mag inconsistent                                  | none                                                         | Hand Over                   	|	v1.0	|
| 13 | Actuator failure               | FailureDetector                 | Actuator output abnormal                          | `FD_ACT_EN`                                                  | Hand over		                |	v1.0	|
| 14 | Invalid Inputs to Controller   | N/A		                | Commander or Offboard abnormal outputs            | none	                                                   | Hand over		                |	v1.0	|
| 15 | Motor Saturation   	      | N/A		                | Motor commands saturated		            | none	                                                   | Parachute		                |	16-04	|
| 16 | Vibration 	   	      | N/A		                | Vibration metrics above threshold		    | none	                                                   | Hold - Attitude	                |	16-04	|


## Safe Action description

| Safe Action      	| Description                      		|
| ---------------- 	| ------------------------------------- 	|
| Hold - Position  	| Hover in place                   		|
| Land - Position  	| Controlled descent               		|
| Hold - Attitude  	| Hover in place | low Descent     		|
| Land - Attitude  	| Controlled descent               		|
| Safe Land - Position 	| Controlled descent to a detectted Pos		|
| Terminate        	| Immediate motor kill             		|
| Parachute deploy 	| Deploy parachute system          		|
| Hold - Position  	| Hover in place                   		|
| CBF		  	| Place velocity to respect a CBF condition    	|
| Hand Over	  	| Hand Over control to Secondary FC   		|

