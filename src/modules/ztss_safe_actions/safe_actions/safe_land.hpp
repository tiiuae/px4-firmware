#pragma once
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_event_trigger.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/ztss_safe_actions_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <px4_platform_common/module_params.h>
#include "safe_action.hpp"

class ZtssActionSafeLandPosition : public ModuleParams, public SafeActionBase
{
	public:
		ZtssActionSafeLandPosition(ZtssSafeActions*, ModuleParams* );
		~ZtssActionSafeLandPosition()=default;

	void start_execution() override;
	void end_execution() override;
	void execute_action() override;


	uint8_t get_action_id()const override;
	uint8_t get_execution_status()const override;

	void process_action_event() override;

	trajectory_setpoint_s trajectory_set_point_;
	uORB::Publication<trajectory_setpoint_s> trajectory_setpoint_pub_{ORB_ID(ztss_trajectory_setpoint)};
};



class ZtssActionSafeLandAttitude : public ModuleParams, public SafeActionBase
{
	public:
		ZtssActionSafeLandAttitude(ZtssSafeActions*,ModuleParams* );
		~ZtssActionSafeLandAttitude()=default;

	void start_execution() override;
	void end_execution() override;
	void execute_action() override;


	uint8_t get_action_id()const override;
	uint8_t get_execution_status()const override;

	void process_action_event() override;


	vehicle_attitude_setpoint_s attitude_set_point_;
	uORB::Publication<vehicle_attitude_setpoint_s> vehicle_attitude_setpoint_pub_{ORB_ID(vehicle_attitude_setpoint)};
};
