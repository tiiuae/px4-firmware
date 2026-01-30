#define DEFINE_GET_PX4_CUSTOM_MODE
#include <commander/px4_custom_mode.h>
#include "safe_hold.hpp"
#include "uORB/topics/vehicle_local_position.h"


ZtssActionSafeHoldPosition::ZtssActionSafeHoldPosition(ZtssSafeActions*parent, ModuleParams* module_params_parent):ModuleParams(module_params_parent),SafeActionBase(parent)
{
	this->action_id= ztss_event_trigger_s::ACTION_HOLD_POSITION;
	this->trajectory_setpoint_pub_.advertise();
};


void ZtssActionSafeHoldPosition::start_execution()
{
	SafeActionBase::start_execution();

	// 2. Construct a trajectory_setpoint_pub_

	const vehicle_local_position_s& vehicle_local_position = SafeActionBase::get_local_position();

	// Hold current position (NED frame)
	trajectory_set_point_.position[0] = vehicle_local_position.x;  // North
	trajectory_set_point_.position[1] = vehicle_local_position.y;  // East
	trajectory_set_point_.position[2] = vehicle_local_position.z;  // Down

	// Zero velocity (hover)
	trajectory_set_point_.velocity[0] = 0.0f;
	trajectory_set_point_.velocity[1] = 0.0f;
	trajectory_set_point_.velocity[2] = 0.0f;

	// No acceleration command (let controller compute)
	trajectory_set_point_.acceleration[0] = NAN;
	trajectory_set_point_.acceleration[1] = NAN;
	trajectory_set_point_.acceleration[2] = NAN;

	// No jerk command
	trajectory_set_point_.jerk[0] = NAN;
	trajectory_set_point_.jerk[1] = NAN;
	trajectory_set_point_.jerk[2] = NAN;

	// Hold current heading
	trajectory_set_point_.yaw = vehicle_local_position.heading;
	trajectory_set_point_.yawspeed = 0.0f;

	trajectory_set_point_.timestamp = hrt_absolute_time();
	trajectory_setpoint_pub_.publish(trajectory_set_point_);

	// 3. Request ZTSS Safe Action mode (Commander treats like Offboard)
	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.0f; // Custom mode
	cmd.param2 = PX4_CUSTOM_MAIN_MODE_ZTSS; // Your custom mode
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 250;
	cmd.confirmation = 0;
	cmd.from_external = false;
	cmd.timestamp = hrt_absolute_time();
	vehicle_command_pub_.publish(cmd);

};

void ZtssActionSafeHoldPosition::execute_action()
{
	SafeActionBase::execute_action();
	// Update the timestamp
	vehicle_control_mode.timestamp = hrt_absolute_time();
	vehicle_control_mode_pub_.publish(vehicle_control_mode);

	trajectory_set_point_.timestamp = hrt_absolute_time();
	this->trajectory_setpoint_pub_.publish(this->trajectory_set_point_);

};

void ZtssActionSafeHoldPosition::end_execution(){
	SafeActionBase::end_execution();
};

void ZtssActionSafeHoldPosition::process_action_event(){
	SafeActionBase::process_action_event();
}

uint8_t ZtssActionSafeHoldPosition::get_action_id()const{return this->action_id;};

uint8_t ZtssActionSafeHoldPosition::get_execution_status()const{return this->execution_status_;};



ZtssActionSafeHoldAttitude::ZtssActionSafeHoldAttitude(ZtssSafeActions*parent, ModuleParams* module_params_parent):ModuleParams(module_params_parent),SafeActionBase(parent)
{
	this->action_id= ztss_event_trigger_s::ACTION_HOLD_ATTITUDE;
	this->_vehicle_attitude_setpoint_pub.advertise();
};


void ZtssActionSafeHoldAttitude::start_execution()
{
	SafeActionBase::start_execution();


	// 2. Create quaternion from heading
	// hover_thrust_estimate_s hover_trust_estimate = SafeActionBase::get_hover_thrust_estimate();
	// vehicle_local_position_s vehicle_local_position = SafeActionBase::get_local_position();

	// Create quaternion from heading (yaw only, level attitude)
	// TODO(renzo) estimate heading with raw gyroscope or use directly vehicle_imu
	// This creates a quaternion for zero roll, zero pitch, and current heading
	// const float heading = vehicle_local_position.heading;
	// const float cos_half_yaw = cosf(heading * 0.5f);
	// const float sin_half_yaw = sinf(heading * 0.5f);

	attitude_set_point_.q_d[0] = 1.0;  // w
	attitude_set_point_.q_d[1] = 0.0f;          // x (roll = 0)
	attitude_set_point_.q_d[2] = 0.0f;          // y (pitch = 0)
	attitude_set_point_.q_d[3] = 0.0f;  // z (yaw from heading)

	// TODO(renzo) estimate thrust to come to a Less smooth stop using raw accelerations or use directly vehicle_imu
	attitude_set_point_.thrust_body[0]= 0.0f;
	attitude_set_point_.thrust_body[1]= 0.0f;
	// TODO(renzo) estimate thrust in the body frame with raw accelerations or use directly vehicle_imu
	attitude_set_point_.thrust_body[2]= -float(0.8) - float(0.005);
	attitude_set_point_.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_set_point_);

	// 3. Request ZTSS Safe Action mode (Commander treats like Offboard)
	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.0f; // Custom mode
	cmd.param2 = PX4_CUSTOM_MAIN_MODE_ZTSS; // Your custom mode
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 250;
	cmd.confirmation = 0;
	cmd.from_external = false;
	cmd.timestamp = hrt_absolute_time();
	vehicle_command_pub_.publish(cmd);

};

void ZtssActionSafeHoldAttitude::execute_action()
{
	SafeActionBase::execute_action();

	// Update timestamps
	vehicle_control_mode.timestamp = hrt_absolute_time();
	this->vehicle_control_mode_pub_.publish(vehicle_control_mode);

	attitude_set_point_.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_set_point_);
};

void ZtssActionSafeHoldAttitude::end_execution()
{
	SafeActionBase::end_execution();
};

void ZtssActionSafeHoldAttitude::process_action_event(){
	SafeActionBase::process_action_event();
}

uint8_t ZtssActionSafeHoldAttitude::get_action_id()const{return this->action_id;};

uint8_t ZtssActionSafeHoldAttitude::get_execution_status()const{return this->execution_status_;};



