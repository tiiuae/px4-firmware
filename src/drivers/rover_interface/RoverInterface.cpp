#include "RoverInterface.hpp"
#include <lib/rover_control/RoverControl.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

RoverInterface *RoverInterface::_instance;

// CAN interface | default is can0
const char *const RoverInterface::CAN_IFACE = "can0";

RoverInterface::RoverInterface(uint8_t rover_type, uint32_t bitrate)
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rover_interface),
	  _rover_type(rover_type),
	  _bitrate(bitrate),
	  _battery(1, nullptr, RoverStatusPublishIntervalMs, battery_status_s::SOURCE_EXTERNAL)
{
	pthread_mutex_init(&_node_mutex, nullptr);
}


RoverInterface::~RoverInterface()
{
	if (_instance) {
		// Tell the task we want it to go away
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 1000;

		do {
			// Wait for it to exit or timeout
			usleep(5000);

			if (--i == 0) {
				PX4_ERR("Failed to Stop Task - reboot needed");
				break;
			}

		} while (_instance);
	}

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}


int RoverInterface::start(uint8_t rover_type, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_ERR("Already started");
		return -1;
	}

	_instance = new RoverInterface(rover_type, bitrate);

	if (_instance == nullptr) {
		PX4_ERR("Failed to allocate RoverInterface object");
		return -1;
	}

	_instance->ScheduleOnInterval(ScheduleIntervalMs);

	return PX4_OK;
}


void RoverInterface::Init()
{
	_initialized = false;

	// Check rover type
	switch (_rover_type) {
	case 0:
		// Scout Mini
		PX4_INFO("Scout Mini (rover type 0) is supported. Initializing...");
		break;

	case 1:
		// Scout
		PX4_INFO("Scout (rover type 1) is not supported. Aborted");
		return;

	case 2:
		// Scout Pro
		PX4_INFO("Scout Pro (rover type 2) is not supported. Aborted");
		return;

	case 3:
		// Scout 2
		PX4_INFO("Scout 2 (rover type 3) is not supported. Aborted");
		return;

	case 4:
		// Scout 2 Pro
		PX4_INFO("Scout 2 Pro (rover type 4) is not supported. Aborted");
		return;

	case 5:
		// Bunker
		PX4_INFO("Bunker (rover type 5) is supported. Initializing...");
		break;

	case 6:
		// Bunker Mini
		PX4_INFO("Bunker Mini (rover type 6) is supported. Initializing...");
		break;

	default:
		// Unknown Rover type
		PX4_INFO("Unknown rover type. Aborted");
		return;
	}

	// Check protocol version and create ScoutRobot object
	if (_protocol_version == scoutsdk::ProtocolVersion::AGX_V1) {
		PX4_INFO("AGX V1 protocol version is not supported. Aborted");

	} else if (_protocol_version == scoutsdk::ProtocolVersion::AGX_V2) {
		PX4_INFO("Detected AGX V2 protocol");
		_scout = new scoutsdk::ScoutRobot(scoutsdk::ProtocolVersion::AGX_V2, true);

	} else {
		PX4_INFO("Unknown protocol version");
	}

	if (_scout == nullptr) {
		PX4_ERR("Failed to create the ScoutRobot object. Aborted");
		return;
	}

	// Finally establish connection to rover
	_scout->Connect(CAN_IFACE, _bitrate);

	if (!_scout->GetCANConnected()) {
		PX4_ERR("Failed to connect to the rover CAN bus");
		return;
	}

	// Use CAN command mode
	_scout->EnableCommandMode();

	// Retrieve system version (blocking)
	_scout->QuerySystemVersion(SystemVersionQueryLimitMs);

	// Setup rover state publisher
	if (!orb_advert_valid(_rover_status_pub)) {
		_rover_status_pub = orb_advertise(ORB_ID(rover_status), &_rover_status_msg);
	}

	// Breathing mode light by default when not armed
	_scout->SetLightCommand(LightMode::BREATH, 0);

	// Allow reception of system status feedback message only
	_scout->AllowOnlySystemStatusFeedback();

	parameters_update(true);
	_initialized = true;
}


void RoverInterface::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_control_parameters_valid = PX4_ISFINITE(_param_ro_max_thr_speed.get())
					    && _param_ro_max_thr_speed.get() > FLT_EPSILON
					    && PX4_ISFINITE(_param_rd_wheel_track.get())
					    && _param_rd_wheel_track.get() > FLT_EPSILON
					    && PX4_ISFINITE(_param_ro_accel_limit.get())
					    && PX4_ISFINITE(_param_ro_decel_limit.get());

		if (!_control_parameters_valid) {
			PX4_ERR("Set positive finite RO_MAX_THR_SPEED, RD_WHEEL_TRACK and finite RO_ACCEL_LIM, RO_DECEL_LIM");
		}
	}
}


void RoverInterface::Run()
{
	pthread_mutex_lock(&_node_mutex);

	if (_instance != nullptr && _task_should_exit.load()) {
		ScheduleClear();

		if (_initialized) {
			StopMotion();
			_initialized = false;
		}

		// Clean up
		if (_scout != nullptr) {
			delete _scout;
			_scout = nullptr;
		}

		_instance = nullptr;
		pthread_mutex_unlock(&_node_mutex);
		return;
	}

	if (_instance != nullptr && !_initialized) {
		// Try initializing for the first time
		if (!_init_try_count) {
			Init();
			_init_try_count++;
		}

		// Return early if still not initialized
		if (!_initialized) {
			pthread_mutex_unlock(&_node_mutex);
			return;
		}
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// Update parameters
	parameters_update();

	// Check for actuator armed command to rover
	ActuatorArmedUpdate();

	// Check for action request (kill switch)
	ActionRequestUpdate();

	// Only the Manual navigation mode uses the manual speed scale.
	VehicleStatusUpdate();

	// Check for motion commands to rover
	if (!_kill_switch && _armed) { RoverSetpointUpdate(); }

	// Check for receive msgs from the rover
	_scout->CheckUpdateFromRover();

	// Update from rover and publish the rover state
	if (hrt_elapsed_time(&_last_rover_status_publish_time) > RoverStatusPublishIntervalMs) {
		PublishRoverState();
	}

	perf_end(_cycle_perf);

	pthread_mutex_unlock(&_node_mutex);
}


void RoverInterface::RoverSetpointUpdate()
{
	const hrt_abstime timestamp_prev = _last_control_update;
	_last_control_update = hrt_absolute_time();
	const float dt = math::constrain(_last_control_update - timestamp_prev, 1_ms, 10_ms) * 1e-6f;

	bool do_update = false;

	if (_rover_steering_setpoint_sub.updated()) {
		rover_steering_setpoint_s rover_steering_setpoint_msg;

		if (_rover_steering_setpoint_sub.copy(&rover_steering_setpoint_msg)) {
			_yaw_control = rover_steering_setpoint_msg.normalized_steering_setpoint;
			_steering_setpoint_timestamp = rover_steering_setpoint_msg.timestamp;
			do_update = true;
		}
	}

	if (_rover_throttle_setpoint_sub.updated()) {
		rover_throttle_setpoint_s rover_throttle_setpoint_msg;

		if (_rover_throttle_setpoint_sub.copy(&rover_throttle_setpoint_msg)) {
			_throttle_control = rover_throttle_setpoint_msg.throttle_body_x;
			_throttle_setpoint_timestamp = rover_throttle_setpoint_msg.timestamp;
			do_update = true;
		}
	}

	// Check both publishers even when no new setpoint arrives, including controller stops.
	if (_throttle_setpoint_timestamp <= _motion_enabled_time ||
	    _steering_setpoint_timestamp <= _motion_enabled_time ||
	    hrt_elapsed_time(&_throttle_setpoint_timestamp) > SetpointTimeout ||
	    hrt_elapsed_time(&_steering_setpoint_timestamp) > SetpointTimeout) {
		if (_setpoints_fresh) {
			PX4_WARN("Rover setpoints stale, stopping");
		}

		_setpoints_fresh = false;
		StopMotion();
		return;
	}

	_setpoints_fresh = true;

	if (!_control_parameters_valid) {
		StopMotion();
		return;
	}

	if (!PX4_ISFINITE(_throttle_control) || !PX4_ISFINITE(_yaw_control)) {
		if (do_update) {
			PX4_WARN("Invalid rover throttle or steering setpoint");
		}

		StopMotion();
		return;
	}

	if (_is_manual_mode && (!PX4_ISFINITE(_param_man_speed_scale.get())
				|| _param_man_speed_scale.get() <= FLT_EPSILON)) {
		if (do_update) {
			PX4_WARN("Set positive finite RI_MAN_SPD_SC");
		}

		StopMotion();
		return;
	}

	const float speed_scale = _is_manual_mode
				  ? math::min(_param_man_speed_scale.get(), _param_ro_max_thr_speed.get())
				  : _param_ro_max_thr_speed.get();
	// Start from the last CAN speed so scale changes and wheel saturation cannot wind up the ramp.
	SlewRate<float> throttle_setpoint{_last_throttle_speed / speed_scale};
	const float adjusted_throttle = RoverControl::throttleControl(throttle_setpoint,
					math::constrain(_throttle_control, -1.f, 1.f), throttle_setpoint.getState(),
					_param_ro_accel_limit.get(), _param_ro_decel_limit.get(), speed_scale, dt);
	const float steering = math::constrain(_yaw_control, -1.f, 1.f);
	// Match DifferentialActControl: preserve steering and reduce throttle at wheel saturation.
	const float throttle_limit = 1.f - fabsf(steering);
	const float throttle = speed_scale * math::constrain(adjusted_throttle, -throttle_limit, throttle_limit);
	// Steering is half the normalized right-minus-left wheel speed, not an angular velocity.
	const float yaw_rate = 2.f * speed_scale * steering / _param_rd_wheel_track.get();

	if (!SendMotionCommand(throttle, yaw_rate)) {
		StopMotion();
		return;
	}

	_last_throttle_speed = throttle;
}


bool RoverInterface::SendMotionCommand(float linear_velocity, float yaw_rate)
{
	if (!_scout->GetCANConnected()) {
		PX4_ERR("Cannot send rover motion command: CAN disconnected");
		return false;
	}

	// The SDK encodes millimetres/milliradians as int16_t and negates yaw.
	const float linear_command = linear_velocity * 1000.f;
	const float angular_command = -yaw_rate * 1000.f;

	if (!PX4_ISFINITE(linear_command) || linear_command < INT16_MIN || linear_command > INT16_MAX
	    || !PX4_ISFINITE(angular_command) || angular_command < INT16_MIN || angular_command > INT16_MAX) {
		PX4_ERR("Non-finite or out-of-range rover motion command");
		return false;
	}

	_scout->SetMotionCommand(linear_velocity, yaw_rate);
	return true;
}


void RoverInterface::StopMotion()
{
	SendMotionCommand(0.f, 0.f);
	_last_throttle_speed = 0.f;
	_last_control_update = hrt_absolute_time();
}


void RoverInterface::ActuatorArmedUpdate()
{
	if (_actuator_armed_sub.updated()) {
		actuator_armed_s actuator_armed_msg;

		if (_actuator_armed_sub.copy(&actuator_armed_msg)) {
			// Arm or disarm the rover
			if (!_armed && actuator_armed_msg.armed) {
				_scout->SetLightCommand(LightMode::CONST_ON, 0);
				_motion_enabled_time = hrt_absolute_time();
				_setpoints_fresh = false;
				_armed = true;

			} else if (_armed && !actuator_armed_msg.armed) {
				StopMotion();
				_scout->SetLightCommand(LightMode::BREATH, 0);
				_armed = false;
			}
		}
	}
}


void RoverInterface::ActionRequestUpdate()
{
	if (_action_request_sub.updated()) {
		action_request_s action_request_msg;

		if (_action_request_sub.copy(&action_request_msg)) {
			// Check for kill switch (expected pub rate is 5Hz)
			switch (action_request_msg.action) {
			case action_request_s::ACTION_KILL:
				_kill_switch = true;
				StopMotion();
				break;

			case action_request_s::ACTION_UNKILL:
				if (_kill_switch) {
					_motion_enabled_time = hrt_absolute_time();
					_setpoints_fresh = false;
				}

				_kill_switch = false;
				break;
			}
		}
	}
}


void RoverInterface::VehicleStatusUpdate()
{
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status_msg;

		if (_vehicle_status_sub.copy(&vehicle_status_msg)) {
			_is_manual_mode = vehicle_status_msg.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL;
		}
	}
}


void RoverInterface::PublishRoverState()
{
	// Get rover state
	auto &robot_state = _scout->GetRobotState();

	// Assign the values to the PX4 side ORB msg
	_rover_status_msg.timestamp = hrt_absolute_time();
	_rover_status_msg.vehicle_state = robot_state.system_state.vehicle_state;
	_rover_status_msg.control_mode = robot_state.system_state.control_mode;
	_rover_status_msg.error_code = robot_state.system_state.error_code;
	_rover_status_msg.battery_voltage = robot_state.system_state.battery_voltage;

	// Update battery status
	if (_param_publish_battery_status.get()) {
		_battery.setConnected(true);
		_battery.updateVoltage(robot_state.system_state.battery_voltage);

		// Calculate remaining percentage based on rover type
		float max_voltage = 27.2f; // Default for most rovers

		if (_rover_type == 5) { // Bunker
			max_voltage = 54.4f;
		}

		// Calculate remaining as percentage (0.0 to 1.0)
		float min_voltage = max_voltage * 0.83f;
		float remaining = 1.0f;

		if (robot_state.system_state.battery_voltage <= min_voltage) {
			remaining = 0.0f;

		} else if (robot_state.system_state.battery_voltage >= max_voltage) {
			remaining = 1.0f;

		} else {
			remaining = (robot_state.system_state.battery_voltage - min_voltage) / (max_voltage - min_voltage);
		}

		// Set the calculated state of charge
		_battery.setStateOfCharge(remaining);

		// Update current (Use default Idle current at 1A since Scout rover doesn't provide current)
		_battery.updateCurrent(1.0f);

		// Update and publish battery status
		_battery.updateAndPublishBatteryStatus(_rover_status_msg.timestamp);
	}

	if (orb_advert_valid(_rover_status_pub)) {
		orb_publish(ORB_ID(rover_status), _rover_status_pub, &_rover_status_msg);
		_last_rover_status_publish_time = _rover_status_msg.timestamp;
	}
}


void RoverInterface::print_status()
{
	pthread_mutex_lock(&_node_mutex);

	if (_scout == nullptr) {
		PX4_ERR("Scout Robot object not initialized");
		pthread_mutex_unlock(&_node_mutex);
		return;
	}

	// CAN connection info
	PX4_INFO("CAN interface: %s. Status: %s",
		 RoverInterface::CAN_IFACE, _scout->GetCANConnected() ? "connected" : "disconnected");

	// Rover info
	if (_scout->GetCANConnected()) {
		PX4_INFO("Rover Type: %d. Protocol Version: %s",
			 _rover_type,
			 _protocol_version == scoutsdk::ProtocolVersion::AGX_V2 ? "AGX_V2" : "Unknown");
		PX4_INFO("Rover system version: %s", _scout->GetSystemVersion());
	}

	// Arm / disarm / kill switch status
	PX4_INFO("Rover is armed: %s. Kill switch: %s", _armed ? "true" : "false", _kill_switch ? "true" : "false");

	// Battery status
	PX4_INFO("Battery status publishing: %s", _param_publish_battery_status.get() ? "enabled" : "disabled");

	// Subscription info
	PX4_INFO("Subscribed to topics: %s, %s, %s",
		 _rover_throttle_setpoint_sub.get_topic()->o_name,
		 _rover_steering_setpoint_sub.get_topic()->o_name,
		 _actuator_armed_sub.get_topic()->o_name);

	// Publication info
	if (orb_advert_valid(_rover_status_pub)) { PX4_INFO("Publishing rover_status topic"); }

	if (_param_publish_battery_status.get()) { PX4_INFO("Publishing battery_status topic"); }

	// Performance counters
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	pthread_mutex_unlock(&_node_mutex);
}


static void print_usage()
{
	PX4_INFO("Usage: \n\trover_interface {start|status|stop}");
}


extern "C" __EXPORT int rover_interface_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (RoverInterface::instance()) {
			PX4_ERR("Already started");
			return 1;
		}

		// Rover type | default is Scout Mini
		int32_t rover_type = 0;
		param_get(param_find("RI_ROVER_TYPE"), &rover_type);

		// Rover interface CAN bitrate | default is 500Kbit/s
		int32_t can_bitrate = 0;
		param_get(param_find("RI_CAN_BITRATE"), &can_bitrate);

		// Start
		PX4_INFO("Start Rover Interface to rover type %" PRId32 " at CAN iface %s with bitrate %" PRId32 " bit/s",
			 rover_type, RoverInterface::CAN_IFACE, can_bitrate);
		return RoverInterface::start(static_cast<uint8_t>(rover_type), can_bitrate);
	}

	/* commands below assume that the app has been already started */
	RoverInterface *const inst = RoverInterface::instance();

	if (!inst) {
		PX4_ERR("Application not running");
		return 1;
	}

	if (!strcmp(argv[1], "status") || !strcmp(argv[1], "info")) {
		inst->print_status();
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
