#include "safe_action.hpp"
#include "ztss_safe_actions/ztss_safe_actions.hpp"




SafeActionBase::SafeActionBase(ZtssSafeActions* parent): parent_safe_actions_(parent)
{
	this->vehicle_control_mode_pub_.advertise();
	this->vehicle_command_pub_.advertise();
};

SafeActionBase::~SafeActionBase() {}


// Default implementation of the process_action_event
void SafeActionBase::process_action_event()
{
	if (this->get_execution_status()==ztss_safe_actions_status_s::START_EXECUTING)
	{
		this->start_execution();
	}
	if (this->get_execution_status()==ztss_safe_actions_status_s::EXECUTING)
	{
		this->execute_action();
	}
	if (this->get_execution_status()==ztss_safe_actions_status_s::END_EXECUTING)
	{
		this->end_execution();
	}
}


bool SafeActionBase::matches_event(const ztss_event_trigger_s & event)
{
	return event.action_id == this->action_id;
}


void SafeActionBase::start_execution()
{
	PX4_INFO("Start Safe Action %s ", action_names[this->action_id]);
	SafeActionBase::set_previous_custom_mode();
	// 1. Publish vehicle_control_mode (Commander checks disabled)
	vehicle_control_mode.timestamp = hrt_absolute_time();
	vehicle_control_mode.flag_armed = true;
	vehicle_control_mode.flag_control_ztss_enabled = true;
	vehicle_control_mode.flag_multicopter_position_control_enabled = true;
	vehicle_control_mode.flag_control_position_enabled = true;
	vehicle_control_mode.flag_control_velocity_enabled = true;
	vehicle_control_mode.flag_control_altitude_enabled = true;
	vehicle_control_mode.flag_control_climb_rate_enabled = true;
	vehicle_control_mode.flag_control_acceleration_enabled = true;
	vehicle_control_mode.flag_control_attitude_enabled = true;
	vehicle_control_mode.flag_control_rates_enabled = true;
	vehicle_control_mode.flag_control_allocation_enabled = true;
	vehicle_control_mode_pub_.publish(vehicle_control_mode);
}


void SafeActionBase::execute_action()
{
}

void SafeActionBase::end_execution()
{
	PX4_INFO("End Execution Safe Action %s ", action_names[this->action_id]);
	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.0f; // custom mode selector
	cmd.param2 = custom_modes_at_ztss_kick_in_.main_mode; // switch back to previous main mode
	cmd.param3 = custom_modes_at_ztss_kick_in_.sub_mode; // restore previous navigation state
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 250;
	cmd.confirmation = 0;
	cmd.from_external = false;

	if(this->get_safe_actions_status().action_id == ztss_event_trigger_s::ACTION_NONE)
	{
		PX4_INFO("Reverting to previous mode: main_mode=%u, sub_mode=%u", custom_modes_at_ztss_kick_in_.main_mode, custom_modes_at_ztss_kick_in_.sub_mode);
		cmd.timestamp = hrt_absolute_time();
		vehicle_command_pub_.publish(cmd);
	}

}

void SafeActionBase::set_previous_custom_mode()
{
	custom_modes_at_ztss_kick_in_ = this->parent_safe_actions_->get_previous_safe_actions_custom_mode();
}


const vehicle_local_position_s& SafeActionBase::get_local_position()const
{
	return this->parent_safe_actions_->vehicle_local_position;
}

const hover_thrust_estimate_s& SafeActionBase::get_hover_thrust_estimate()const
{
	return this->parent_safe_actions_->hover_thrust_estimate;
}

const ztss_safe_actions_status_s& SafeActionBase::get_safe_actions_status()const
{
	return this->parent_safe_actions_->ztss_safe_actions_status;
}

uint8_t SafeActionBase::get_action_id() const { return ztss_event_trigger_s::ACTION_NONE; }
uint8_t SafeActionBase::get_execution_status() const { return ztss_safe_actions_status_s::NOT_EXECUTING; }
void SafeActionBase::set_execution_status(uint8_t status)
{
	this->execution_status_ = status;
}

void SafeActionBase::set_prepare_for_crash(bool prepare_for_crash)
{
	this->parent_safe_actions_->prepare_for_crash_ = prepare_for_crash;
}


