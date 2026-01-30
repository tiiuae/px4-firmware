#pragma once
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_event_trigger.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/ztss_safe_actions_status.h>
#include <px4_platform_common/module_params.h>
#include "helpers/actions.hpp"
#include "safe_action.hpp"
#include "uORB/topics/trajectory_setpoint.h"
#include "uORB/topics/vehicle_attitude_setpoint.h"
#include "uORB/topics/hover_thrust_estimate.h"
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>


// Summary: ZTSS Safe Hover Interface
// Approach	Topic	Bypasses	Best for
// Trajectory setpoint	trajectory_setpoint	Nothing (uses full cascade)	Position estimate healthy, want altitude hold
// Attitude setpoint	vehicle_attitude_setpoint	Position controller	Position invalid, need guaranteed response


/*
publishes to :
trajectory_setpoint_s
_trajectory_setpoint_sub.update(&_setpoint);
if Position is ok!


published to :
vehicle_attitude_setpoint
if EKF roll pitch and yaw are ok


RTA should escalate without modes:

Clamp velocity
Clamp tilt
Freeze XY
Force vertical profile
Only then: trigger LAND / DISARM

Commander is used only at the final step.

*/

class ZtssActionSafeHoldPosition : public ModuleParams, public SafeActionBase
{
	public:
		ZtssActionSafeHoldPosition(ZtssSafeActions* , ModuleParams* );
		~ZtssActionSafeHoldPosition()=default;

	void start_execution() override;
	void end_execution() override;
	void execute_action() override;


	uint8_t get_action_id()const override;
	uint8_t get_execution_status()const override;


	void process_action_event() override;

	trajectory_setpoint_s trajectory_set_point_;
	uORB::Publication<trajectory_setpoint_s> trajectory_setpoint_pub_{ORB_ID(ztss_trajectory_setpoint)};
};


class ZtssActionSafeHoldAttitude : public ModuleParams, public SafeActionBase
{

	public:
	ZtssActionSafeHoldAttitude(ZtssSafeActions*, ModuleParams* );
	~ZtssActionSafeHoldAttitude()=default;

	void start_execution() override;
	void end_execution() override;
	void execute_action() override;


	uint8_t get_action_id()const override;
	uint8_t get_execution_status()const override;

	void process_action_event() override;

	vehicle_attitude_setpoint_s attitude_set_point_;
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

};


