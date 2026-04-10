| #  | Safety Case                    | Module Detecting It             | Trigger Condition                                 | Parameter(s)                                                 | Safe Action                      |
| -- | ------------------------------ | ------------------------------- | ------------------------------------------------- | ------------------------------------------------------------ | -------------------------------- |
| 1  | Attitude failure (flip/tumble) | FailureDetector                 | Roll or pitch exceeds threshold for time window   | `FD_FAIL_R`, `FD_FAIL_P`, `FD_FAIL_R_TTRI`, `FD_FAIL_P_TTRI` | Flight termination or disarm     |
| 2  | External ATS trigger           | FailureDetector                 | PWM signal from external automatic trigger system | `FD_EXT_ATS_EN`, `FD_EXT_ATS_TRIG`                           | Flight termination               |
| 3  | Motor / ESC failure            | FailureDetector + ESC telemetry | ESC telemetry reports error or timeout            | `FD_ACT_EN`                                                  | Terminate or emergency landing   |
| 4  | RC signal loss                 | Commander                       | No RC input for timeout                           | `COM_RC_LOSS_T`, `NAV_RCL_ACT`                               | Hold / RTL / Land / Terminate    |
| 5  | GCS datalink loss              | Commander                       | MAVLink heartbeat lost                            | `COM_DL_LOSS_T`, `NAV_DLL_ACT`                               | Hold / RTL / Land                |
| 6  | Offboard control loss          | Commander                       | No offboard setpoints                             | `COM_OF_LOSS_T`, `COM_OBL_ACT`                               | Switch to position control / RTL |
| 7  | Low battery                    | Commander + Battery module      | Battery voltage or capacity below threshold       | `BAT_LOW_THR`                                                | Warning                          |
| 8  | Critical battery               | Commander                       | Battery reaches critical level                    | `BAT_CRIT_THR`                                               | RTL                              |
| 9  | Emergency battery              | Commander                       | Battery extremely low                             | `BAT_EMERGEN_THR`                                            | Immediate land                   |
| 10 | Position estimate lost         | Commander + EKF                 | `vehicle_local_position.valid = false`            | none                                                         | Mode downgrade / land            |
| 11 | Velocity estimate lost         | EKF + Commander                 | Velocity validity lost                            | none                                                         | Mode downgrade                   |
| 12 | GPS failure                    | EKF2                            | Innovation errors / loss of GPS data              | `EKF2_*` parameters                                          | Switch navigation mode / land    |
| 13 | EKF divergence                 | EKF2                            | Innovation test ratio too large                   | `EKF2_*` thresholds                                          | Prevent arming or failsafe       |
| 14 | Geofence violation             | Navigator                       | Vehicle outside allowed area                      | `GF_ACTION`                                                  | RTL / Land / Terminate           |
| 15 | Wind limit exceeded            | Commander                       | Wind estimate exceeds threshold                   | `COM_WIND_MAX`, `COM_WIND_MAX_ACT`                           | RTL / Land                       |
| 16 | Mission failure                | Navigator                       | Waypoint unreachable or mission error             | `NAV_*`                                                      | RTL                              |
| 17 | VTOL transition failure        | VTOL module                     | Transition timeout or failure                     | `VT_*`                                                       | Abort transition                 |
| 18 | Sensor failure                 | Sensors module                  | Sensor missing or unhealthy                       | various                                                      | Sensor switch or arming denial   |
| 19 | IMU failure                    | Sensors + estimator             | IMU health check fails                            | none                                                         | Switch IMU / disarm              |
| 20 | Barometer failure              | Sensors                         | Barometer invalid                                 | none                                                         | Switch sensor                    |
| 21 | Magnetometer failure           | Sensors                         | Mag inconsistent                                  | none                                                         | Switch sensor                    |
| 22 | Manual kill switch             | Commander                       | RC kill switch activated                          | `RC_MAP_KILL_SW`                                             | Immediate motor stop             |
| 23 | Safety switch not pressed      | Commander                       | Hardware safety button engaged                    | none                                                         | Prevent arming                   |
| 24 | Preflight arming failure       | Commander health checks         | Required checks fail                              | `COM_ARM_*`                                                  | Deny arming                      |
| 25 | Flight termination request     | Commander                       | FailureDetector or external trigger               | `CBRK_FLIGHTTERM`                                            | Force failsafe outputs           |
| 26 | Actuator failure               | FailureDetector                 | Actuator output abnormal                          | `FD_ACT_EN`                                                  | Terminate or land                |
| 27 | Parachute trigger              | Commander / FailureDetector     | catastrophic event detected                       | `PARACHUTE_*`                                                | Deploy parachute                 |


<!-- | #  | Safety Case                    | Detecting Module           | Main File(s)                           | Key Function(s)             | Trigger Condition                                                  | Resulting Safe Action          |
| -- | ------------------------------ | -------------------------- | -------------------------------------- | --------------------------- | ------------------------------------------------------------------ | ------------------------------ |
| 1  | Attitude failure (flip/tumble) | FailureDetector            | `failure_detector/FailureDetector.cpp` | `updateAttitudeStatus()`    | Roll or pitch exceeds `FD_FAIL_R`, `FD_FAIL_P` for configured time | Flight termination or disarm   |
| 2  | External ATS trigger           | FailureDetector            | `failure_detector/FailureDetector.cpp` | `updateExternalAtsStatus()` | External PWM trigger above `FD_EXT_ATS_TRIG`                       | Flight termination             |
| 3  | Actuator / motor failure       | FailureDetector            | `failure_detector/FailureDetector.cpp` | `updateActuatorStatus()`    | Actuator output anomaly or telemetry fault                         | Terminate or emergency landing |
| 4  | RC signal loss                 | Commander                  | `Commander.cpp`                        | `checkRcLoss()`             | No RC updates for `COM_RC_LOSS_T`                                  | Hold / RTL / Land / Terminate  |
| 5  | GCS datalink loss              | Commander                  | `Commander.cpp`                        | `checkDataLinkLoss()`       | MAVLink heartbeat timeout (`COM_DL_LOSS_T`)                        | Hold / RTL / Land              |
| 6  | Offboard control loss          | Commander                  | `Commander.cpp`                        | `checkOffboardLoss()`       | Offboard setpoints stop for `COM_OF_LOSS_T`                        | Switch to POSCTL / RTL         |
| 7  | Low battery                    | Commander + battery module | `Commander.cpp`                        | `batteryStatusUpdate()`     | Battery warning `BAT_LOW_THR`                                      | Warning only                   |
| 8  | Critical battery               | Commander                  | `Commander.cpp`                        | `batteryStatusUpdate()`     | `BAT_CRIT_THR` reached                                             | RTL                            |
| 9  | Emergency battery              | Commander                  | `Commander.cpp`                        | `batteryStatusUpdate()`     | `BAT_EMERGEN_THR` reached                                          | Immediate land                 |
| 10 | Position estimate loss         | Commander + estimator      | `Commander.cpp`                        | `checkPosVelValidity()`     | `vehicle_local_position.xy_valid=false`                            | Mode downgrade / hold          |
| 11 | Velocity estimate loss         | Estimator + Commander      | `Commander.cpp`                        | `checkPosVelValidity()`     | Invalid velocity estimate                                          | Mode downgrade                 |
| 12 | GPS failure                    | Estimator (EKF2)           | `ekf2/`                                | EKF innovation checks       | GPS measurement failure                                            | Mode downgrade / land          |
| 13 | EKF divergence                 | Estimator                  | `ekf2/` + `Commander.cpp`              | estimator health checks     | Innovation test ratio exceeded                                     | Deny arming or failsafe        |
| 14 | Geofence violation             | Navigator                  | `navigator/geofence.cpp`               | `Geofence::check()`         | Vehicle leaves defined boundary                                    | RTL / Land / Terminate         |
| 15 | Wind limit exceeded            | Commander                  | `Commander.cpp`                        | `checkWindConditions()`     | Wind estimate exceeds `COM_WIND_MAX`                               | Warn / RTL / Land              |
| 16 | Mission failure                | Navigator                  | `navigator/mission.cpp`                | `Mission::run()`            | Waypoint unreachable or mission error                              | RTL                            |
| 17 | VTOL transition failure        | VTOL module                | `vtol_att_control/`                    | transition logic            | Transition timeout                                                 | Abort transition               |
| 18 | Sensor failure                 | Sensors module             | `sensors/`                             | sensor health checks        | Sensor missing or invalid                                          | Switch sensor / deny arming    |
| 19 | IMU failure                    | Sensors / estimator        | `sensors/`                             | sensor validation           | IMU unhealthy                                                      | Switch IMU / disarm            |
| 20 | Barometer failure              | Sensors                    | `sensors/`                             | sensor validation           | Barometer invalid                                                  | Sensor fallback                |
| 21 | Magnetometer failure           | Sensors                    | `sensors/`                             | sensor validation           | Mag inconsistency                                                  | Sensor fallback                |
| 22 | Manual kill switch             | Commander                  | `Commander.cpp`                        | command handling            | RC kill switch (`RC_MAP_KILL_SW`)                                  | Immediate disarm               |
| 23 | Hardware safety switch         | Commander                  | `Commander.cpp`                        | safety state checks         | Safety button engaged                                              | Prevent arming                 |
| 24 | Preflight health check failure | Commander                  | `HealthAndArmingChecks/`               | `PreFlightCheck`            | Sensor / estimator / config check fails                            | Arming denied                  |
| 25 | Flight termination request     | Commander                  | `Commander.cpp`                        | `handleFlightTermination()` | FailureDetector or external trigger                                | Motor kill / parachute         |
| 26 | Actuator failure flag          | FailureDetector            | `FailureDetector.cpp`                  | `updateActuatorStatus()`    | Motor or control anomaly                                           | Terminate or land              |
| 27 | Parachute deployment           | Commander                  | `Commander.cpp`                        | termination handling        | Catastrophic failure or manual trigger                             | Deploy parachute               | -->



| Safe Action      | Description                      |
| ---------------- | -------------------------------- |
| Warning          | Notify pilot only                |
| Mode downgrade   | e.g. Position → Altitude control |
| Hold             | Hover in place                   |
| RTL              | Return to launch                 |
| Land             | Controlled descent               |
| Disarm           | Stop motors                      |
| Terminate        | Immediate motor kill             |
| Parachute deploy | Deploy parachute system          |
| Deny arming      | Prevent flight                   |



                   +----------------------+
                   |      Sensors         |
                   | IMU / GPS / Mag / Baro
                   +----------+-----------+
                              |
                              v
                     +------------------+
                     |   Estimator      |
                     |    EKF2          |
                     +---------+--------+
                               |
                publishes state validity
                               |
                               v
                     +------------------+
                     | FailureDetector  |
                     |  (Commander)     |
                     +---------+--------+
                               |
                 catastrophic failure flags
                               |
                               v
+--------------------------------------------------+
|                 Commander Module                  |
|                                                  |
|  Handles most failsafe logic:                    |
|                                                  |
|  - RC loss                                       |
|  - Datalink loss                                 |
|  - Offboard loss                                 |
|  - Battery failsafe                              |
|  - Wind failsafe                                 |
|  - Health checks                                 |
|                                                  |
+--------------------+-----------------------------+
                     |
                     v
             +------------------+
             |   Navigator      |
             |                  |
             | Mission logic    |
             | Geofence         |
             | RTL              |
             +---------+--------+
                       |
                       v
              +----------------+
              | Flight Mode    |
              | State Machine  |
              +--------+-------+
                       |
                       v
              +----------------+
              | Controllers    |
              | Position       |
              | Attitude       |
              +--------+-------+
                       |
                       v
                +-------------+
                |  Mixer      |
                | Actuators   |
                +-------------+


RC input stops
      ↓
Commander detects timeout
      ↓
Failsafe triggered
      ↓
Mode change → RTL
      ↓
Navigator executes return
      ↓
Position controller generates commands
      ↓
Vehicle flies home


Attitude exceeds limit
      ↓
FailureDetector triggered
      ↓
Commander sets flight termination
      ↓
actuator_armed.force_failsafe = true
      ↓
Motors cut / parachute deploy


| Layer           | Purpose                      |
| --------------- | ---------------------------- |
| Sensors         | hardware health              |
| Estimator       | state validity               |
| FailureDetector | catastrophic flight failures |
| Commander       | operational failsafes        |
| Navigator       | mission/geofence safety      |


FailureDetector handles only catastrophic failures.
Everything else is handled by Commander failsafes.
