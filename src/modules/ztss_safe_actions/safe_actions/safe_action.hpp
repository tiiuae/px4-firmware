#pragma once
#include <uORB/Publication.hpp>
#include <px4_platform_common/module_params.h>
#include "uORB/topics/ztss_event_trigger.h"
#include <uORB/topics/ztss_safe_actions_status.h>
#include "helpers/actions.hpp"   // Action, ExecutionStatus, action_names, ...
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include "commander/px4_custom_mode.h"
#include <uORB/topics/ztss_safe_actions_status.h>


class ZtssSafeActions;
struct vehicle_local_position_s;
struct hover_thrust_estimate_s;

class SafeActionBase
{
	public:
	SafeActionBase()=delete;
	SafeActionBase(ZtssSafeActions*);
	virtual ~SafeActionBase();

	virtual void process_action_event();

	bool matches_event(const ztss_event_trigger_s &);

	virtual uint8_t get_action_id() const;
	virtual uint8_t get_execution_status() const;

	const vehicle_local_position_s& get_local_position()const;

	const hover_thrust_estimate_s& get_hover_thrust_estimate()const;


	const ztss_safe_actions_status_s& get_safe_actions_status()const;

	void set_previous_custom_mode();

	void set_execution_status(uint8_t);

	void set_prepare_for_crash(bool prepare_for_crash);


	protected:

	uint8_t execution_status_{ztss_safe_actions_status_s::NOT_EXECUTING};
	uint8_t action_id{ztss_event_trigger_s::ACTION_NONE};


	virtual void start_execution();
	virtual void end_execution();
	virtual void execute_action();

	ZtssSafeActions* parent_safe_actions_;

	vehicle_command_s vehicle_command_{};
	vehicle_control_mode_s vehicle_control_mode{};
	px4_custom_mode custom_modes_at_ztss_kick_in_{};



	uORB::Publication<vehicle_control_mode_s> vehicle_control_mode_pub_{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_command_s> vehicle_command_pub_{ORB_ID(vehicle_command)};



};



