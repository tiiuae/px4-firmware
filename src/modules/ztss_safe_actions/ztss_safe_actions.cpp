#include "ztss_safe_actions.hpp"
#include <inttypes.h>

ZtssSafeActions::ZtssSafeActions(): ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{

	PX4_INFO("Ztss Safe Actions Constructor");
	// Initialize array with pointers to dynamically allocated objects
	safe_actions_[0] = new ZtssActionNone(this, this);
	safe_actions_[1] = new ZtssActionSafeHoldPosition(this, this);
	safe_actions_[2] = new ZtssActionSafeHoldAttitude(this, this);
	safe_actions_[3] = new ZtssActionParachute(this,this);

	PX4_INFO("Ztss Safe Actions Created");


	execution_status_pub_.advertise();
	actuator_armed_pub_.advertise();
};


ZtssSafeActions::~ZtssSafeActions()
{
    // Clean up dynamically allocated safe actions
    for (auto* action : safe_actions_) {
        delete action;
    }
}

int ZtssSafeActions::task_spawn(int argc, char * argv[])
{
	ZtssSafeActions* instance = new ZtssSafeActions();

	if (instance == nullptr)
	{
		PX4_ERR("Ztss Safe Actions Allocation Failed");
		return PX4_ERROR;
	}

	// Register the instance with ModuleBase
	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init())
	{
		PX4_ERR("Ztss Safe Actions Init Failed");
		delete instance;
		_object.store(nullptr);
		return PX4_ERROR;
	}

	PX4_INFO("Ztss Safe Actions Module Started");
	return PX4_OK;
}

int ZtssSafeActions::custom_command(int argc, char* argv[])
{
	return print_usage("unknown command");
};

int ZtssSafeActions::print_usage(const char* reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The module has two action types: Soft Safety and Hard Safety.
Soft Safety will use the baseline cascade controller after having evaluated it is functional.
Hard Safety will trigger actions that handle the control to a different flight controller, to kill motors or to trigger the parachute.

ZTSS safe actions will run at 100 hz to evaluate the events coming from the decision engine to output soft safety action and Hard safety.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ztss_safe_actions", "ztss_safe_controllers");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
};


void ZtssSafeActions::Run()
{
	if(should_exit())
	{
		ztss_event_sub_.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// Reshedule the Run() call
	// High-frequency scheduling for continuous setpoint publishing
	// 125 Hz = 8ms interval (matches position controller rate)
	ScheduleDelayed(8_ms);

	// Update state only if new event arrived (avoid processing stale events)
	if (ztss_event_sub_.updated()) {
		ztss_event_sub_.copy(&event_trigger);
	}

	if (trajectory_setpoint_sub_.updated()){
		trajectory_setpoint_sub_.copy(&placeholder_incoming_trajectory_setpoint_);
	}

	if (vehicle_status_sub_.updated()){
		vehicle_status_sub_.copy(&vehicle_status);
	}



	// Update vehicle state
	local_pos_sub_.copy(&vehicle_local_position);
	hover_thrust_estimate_sub_.update(&hover_thrust_estimate);

	// update the main mode and submodes if the commander is in control.
	// This is needed to be able to restore the previous mode after a safe action is triggered.
	if(ztss_safe_actions_status.action_id == ztss_event_trigger_s::ACTION_NONE)
	{
		previous_safe_actions_custom_mode_ = get_px4_custom_mode(vehicle_status.nav_state); // Store the current custom mode before any safe action modifies it
	}

	// Process all safe actions
	for(size_t idx = 0; idx < NUMBER_OF_SAFE_ACTIONS; idx++)
	{
		if (safe_actions_[idx]->matches_event(this->event_trigger)) {
			// Event matches this action
			ztss_safe_actions_status.timestamp = hrt_absolute_time();
			ztss_safe_actions_status.action_id = safe_actions_[idx]->get_action_id();
			ztss_safe_actions_status.execution_status = safe_actions_[idx]->get_execution_status();

			if(safe_actions_[idx]->get_action_id() == ztss_event_trigger_s::ACTION_NONE) continue;

			// State machine: transition through states
			switch (safe_actions_[idx]->get_execution_status())
			{
				case ztss_safe_actions_status_s::NOT_EXECUTING:
					// Transition to START_EXECUTING
					safe_actions_[idx]->set_execution_status(ztss_safe_actions_status_s::START_EXECUTING);
					break;

				case ztss_safe_actions_status_s::START_EXECUTING:
					// After start_execution() is called, transition to EXECUTING
					safe_actions_[idx]->set_execution_status(ztss_safe_actions_status_s::EXECUTING);
					break;

				case ztss_safe_actions_status_s::EXECUTING:
					// Continue executing (stays in EXECUTING)
					break;

				default:
					break;
			}
		} else {
			// Event doesn't match - end execution if currently active
			if (safe_actions_[idx]->get_execution_status() == ztss_safe_actions_status_s::EXECUTING) {
				safe_actions_[idx]->set_execution_status(ztss_safe_actions_status_s::END_EXECUTING);
			}
		}

		// Process the action based on current state (this calls start/execute/end methods)
		safe_actions_[idx]->process_action_event();

		// After processing, handle state transitions for END_EXECUTING
		if (safe_actions_[idx]->get_execution_status() == ztss_safe_actions_status_s::END_EXECUTING) {
			safe_actions_[idx]->set_execution_status(ztss_safe_actions_status_s::NOT_EXECUTING);
		}
	}

	// Publish execution status
	execution_status_pub_.publish(ztss_safe_actions_status);

	if (ztss_safe_actions_status.action_id == ztss_event_trigger_s::ACTION_NONE)
	{
		filter_trajectory_setpoint();
		trajectory_setpoint_pub_.publish(safe_trajectory_setpoint_);
	}

	// arm the actuators
	actuator_armed_.timestamp = hrt_absolute_time();
	actuator_armed_.armed = true; //TODO(renzo) Disarm if landed, arm otherwise. use accelerometers
	actuator_armed_.force_failsafe= false;
	actuator_armed_.in_esc_calibration_mode= false;
	actuator_armed_.ready_to_arm = true;
	actuator_armed_.lockdown = false;
	actuator_armed_.manual_lockdown = false;
	actuator_armed_.prearmed = false;

	// Control the arming process of the motors
	if(vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ZTSS && prepare_for_crash_)
	{
		actuator_armed_.armed = false;
		actuator_armed_pub_.publish(actuator_armed_);
	}

}

void ZtssSafeActions::filter_trajectory_setpoint()
{
	safe_trajectory_setpoint_ = placeholder_incoming_trajectory_setpoint_;
}

bool ZtssSafeActions::init()
{
	if(!ztss_event_sub_.registerCallback())
	{
		PX4_ERR("callback registration failed");
		return false;
	}

	ScheduleNow();

	return true;

}

const px4_custom_mode ZtssSafeActions::get_previous_safe_actions_custom_mode()const
{
	return previous_safe_actions_custom_mode_;
}


int ZtssSafeActions::print_status()
{
	PX4_INFO("=== ZTSS Safe Actions Status ===");
	PX4_INFO("Module running at 125 Hz (8ms interval)");
	PX4_INFO("");

	// Print current event trigger
	PX4_INFO("Current Event Trigger:");
	PX4_INFO("  Action ID: %u (%s)", event_trigger.action_id,
	         event_trigger.action_id < sizeof(action_names)/sizeof(action_names[0])
	         ? action_names[event_trigger.action_id] : "UNKNOWN");
	PX4_INFO("  Timestamp: %" PRIu64, event_trigger.timestamp);
	PX4_INFO("");

	// Print current execution status
	PX4_INFO("Current Execution Status:");
	PX4_INFO("  Action ID: %u (%s)", ztss_safe_actions_status.action_id,
	         ztss_safe_actions_status.action_id < sizeof(action_names)/sizeof(action_names[0])
	         ? action_names[ztss_safe_actions_status.action_id] : "UNKNOWN");
	PX4_INFO("  Status: %u (%s)", ztss_safe_actions_status.execution_status,
	         ztss_safe_actions_status.execution_status < sizeof(execution_status_names)/sizeof(execution_status_names[0])
	         ? execution_status_names[ztss_safe_actions_status.execution_status] : "UNKNOWN");
	PX4_INFO("  Timestamp: %" PRIu64, ztss_safe_actions_status.timestamp);
	PX4_INFO("");

	// Print vehicle state
	PX4_INFO("Vehicle State:");
	PX4_INFO("  Position (NED): [%.2f, %.2f, %.2f] m",
	         (double)vehicle_local_position.x,
	         (double)vehicle_local_position.y,
	         (double)vehicle_local_position.z);
	PX4_INFO("  Velocity (NED): [%.2f, %.2f, %.2f] m/s",
	         (double)vehicle_local_position.vx,
	         (double)vehicle_local_position.vy,
	         (double)vehicle_local_position.vz);

	PX4_INFO("  Hover Thrust: %.3f", (double)hover_thrust_estimate.hover_thrust);
	PX4_INFO("");

	// Print individual safe action states
	PX4_INFO("Safe Actions Status:");
	for (size_t idx = 0; idx < NUMBER_OF_SAFE_ACTIONS; idx++) {
		if (safe_actions_[idx] != nullptr) {
			uint8_t action_id = safe_actions_[idx]->get_action_id();
			uint8_t exec_status = safe_actions_[idx]->get_execution_status();

			PX4_INFO("  [%zu] %s: %s",
			         idx,
			         action_id < sizeof(action_names)/sizeof(action_names[0])
			         ? action_names[action_id] : "UNKNOWN",
			         exec_status < sizeof(execution_status_names)/sizeof(execution_status_names[0])
			         ? execution_status_names[exec_status] : "UNKNOWN");
		}
	}
	PX4_INFO("");

	// Print subscription status
	PX4_INFO("Subscriptions:");
	PX4_INFO("  ztss_event: %s", ztss_event_sub_.registered() ? "registered" : "not registered");
	PX4_INFO("");

	PX4_INFO("Publications:");
	PX4_INFO("  ztss_safe_actions_status: advertised");
	PX4_INFO("");

	return 0;
}

extern "C" __EXPORT int ztss_safe_actions_main(int argc, char* argv[]){return ZtssSafeActions::main(argc, argv);}
