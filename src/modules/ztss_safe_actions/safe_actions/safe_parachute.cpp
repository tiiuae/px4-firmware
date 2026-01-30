#include "safe_parachute.hpp"


ZtssActionParachute::ZtssActionParachute(ZtssSafeActions* parent , ModuleParams* module_params_parent): ModuleParams(module_params_parent), SafeActionBase(parent)
{
	this->action_id = ztss_event_trigger_s::ACTION_PARACHUTE;
	servo_publisher_.advertise();
}

void ZtssActionParachute::start_execution()
{
	SafeActionBase::start_execution();
	// 1. Request ZTSS Safe Action mode (Commander treats like Offboard)
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

	// extra
	this->set_prepare_for_crash(true);

}

void ZtssActionParachute::execute_action()
{
	// 1.  Update timestamp

	// 2. Set the servos activation
	actuator_servos_s actuator_servos_data{};
	actuator_servos_data.timestamp = hrt_absolute_time();
	actuator_servos_data.control[0] = 1.0f;
	actuator_servos_data.control[1] = 1.0f;
	actuator_servos_data.control[2] = 1.0f;
	actuator_servos_data.control[3] = 1.0f;
	actuator_servos_data.control[4] = 1.0f;
	actuator_servos_data.control[5] = 1.0f;
	actuator_servos_data.control[6] = 1.0f;
	actuator_servos_data.control[7] = 1.0f;


	// 3. publish actuator servos setpoint
	this->servo_publisher_.publish(actuator_servos_data);
}


void ZtssActionParachute::end_execution()
{
	SafeActionBase::end_execution();
		// 2. Set the servos activation
	actuator_servos_s actuator_servos_data{};
	actuator_servos_data.timestamp = hrt_absolute_time();
	actuator_servos_data.control[0] = 0.0f;
	actuator_servos_data.control[1] = 0.0f;
	actuator_servos_data.control[2] = 0.0f;
	actuator_servos_data.control[3] = 0.0f;
	actuator_servos_data.control[4] = 0.0f;
	actuator_servos_data.control[5] = 0.0f;
	actuator_servos_data.control[6] = 0.0f;
	actuator_servos_data.control[7] = 0.0f;


	// 3. publish actuator servos setpoint
	this->servo_publisher_.publish(actuator_servos_data);
	this->set_prepare_for_crash(false);
};
void ZtssActionParachute::process_action_event()
{
	SafeActionBase::process_action_event();
}

uint8_t ZtssActionParachute::get_action_id() const
{
	return this->action_id;
}

uint8_t ZtssActionParachute::get_execution_status() const
{
	return this->execution_status_;
}
