#pragma once
#define DEFINE_GET_PX4_CUSTOM_MODE
#include <commander/px4_custom_mode.h>

#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/critical_action.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "safe_actions/safe_action_none.hpp"
#include "safe_actions/safe_hold.hpp"
#include "safe_actions/safe_land.hpp"
#include "safe_actions/safe_parachute.hpp"
#include "safe_actions/safe_kill_motors.hpp"
#include "safe_actions/safe_safe_land.hpp"
#include "safe_actions/safe_return_to_launch.hpp"
#include <uORB/topics/ztss_event_trigger.h>
#include <uORB/topics/ztss_safe_actions_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <atomic>

#include <array>



// | Mode                                  | Does it bypass part of the cascade?              | What runs                                  |
// | ------------------------------------- | ------------------------------------------------ | ------------------------------------------ |
// | **Acro (manual rate)**                | ✔ bypass position & velocity                     | Attitude + body rate                       |
// | **Stabilized (e.g., Roll/Pitch/Yaw)** | ✔ bypass position & velocity                     | Attitude + body rate                       |
// | **Manual**                            | ✔ bypass all except body–rate control            | Body rate only                             |
// | **Altitude (ALTCTL)**                 | ✖ uses altitude hold, but no position XY control | Altitude controller + attitude + body rate |
// | **Position (POSCTL)**                 | ✖ full cascade                                   | Position + velocity + attitude + body rate |
// | **Auto / Mission**                    | ✖ full cascade                                   | Trajectory + all below                     |


// | Flight Mode     | Bypasses Position | Bypasses Velocity | Bypasses Attitude | Publishes Trajectory?   |
// | --------------- | ----------------- | ----------------- | ----------------- | ----------------------- |
// | Manual/Acro     | Yes               | Yes               | No                | No                      |
// | Stabilized      | Yes               | Yes               | No                | No                      |
// | Altitude        | Yes (XY)          | No                | No                | No (just altitude)      |
// | Velocity        | Yes (pos)         | No                | No                | Partial (velocity only) |
// | Position / Auto | No                | No                | No                | Yes                     |
// | Offboard        | Depends           | Depends           | Depends           | Depends                 |



constexpr size_t NUMBER_OF_SAFE_ACTIONS=4;

using namespace time_literals;


class ZtssSafeActions : public ModuleBase<ZtssSafeActions>, public ModuleParams, public px4::ScheduledWorkItem
{
public:


	ZtssSafeActions();
	~ZtssSafeActions();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	// void run() override;

	/** @see ScheduledWorkItem::Run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

	void filter_trajectory_setpoint();

	const px4_custom_mode get_previous_safe_actions_custom_mode()const;

	// Placeholder for incoming messages used in the safe actions
	vehicle_local_position_s 						vehicle_local_position{};
	hover_thrust_estimate_s 						hover_thrust_estimate{};
	vehicle_status_s							vehicle_status{};
	ztss_safe_actions_status_s 						ztss_safe_actions_status{};
	std::atomic<bool>							prepare_for_crash_{false};




private:
	// Subscriptions
	uORB::SubscriptionCallbackWorkItem 					ztss_event_sub_{this,ORB_ID(ztss_event)};
	uORB::Subscription							vehicle_status_sub_{ORB_ID(vehicle_status)};
	uORB::Subscription 							local_pos_sub_{ORB_ID(vehicle_local_position)};
	uORB::Subscription 							hover_thrust_estimate_sub_{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription							trajectory_setpoint_sub_{ORB_ID(trajectory_setpoint)};

	// Publications
	uORB::Publication<trajectory_setpoint_s>				trajectory_setpoint_pub_{ORB_ID(ztss_trajectory_setpoint)};
	uORB::Publication<ztss_safe_actions_status_s> 				execution_status_pub_{ORB_ID(ztss_safe_action_status)};
	uORB::Publication<actuator_armed_s> 					actuator_armed_pub_{ORB_ID(actuator_armed)};


	// Placeholders for current state and events
	ztss_event_trigger_s 							event_trigger{};
	actuator_armed_s 							actuator_armed_{};
	trajectory_setpoint_s 							placeholder_incoming_trajectory_setpoint_{};
	trajectory_setpoint_s							safe_trajectory_setpoint_{};
	px4_custom_mode 							previous_safe_actions_custom_mode_{};

	// Resources
	std::array<SafeActionBase*, NUMBER_OF_SAFE_ACTIONS> 			safe_actions_;

};



// ┌──────────────────────────────────────────────────────┐
// │                   Ground Station                      │
// │          (requests mode: EXTERNAL3, etc.)             │
// └────────────────────┬─────────────────────────────────┘
//                      │ MAVLink / vehicle_command
//                      ↓
// ┌──────────────────────────────────────────────────────┐
// │                    Commander                          │
// │  ┌────────────────────────────────────────────────┐  │
// │  │           ModeManagement                       │  │
// │  │  • Tracks registered external modes            │  │
// │  │  • Decides which executor is "in charge"       │  │
// │  │  • Applies control mode config                 │  │
// │  │  • Handles mode replacement                    │  │
// │  └────────────────────────────────────────────────┘  │
// │                                                       │
// │  handle_command() checks with ModeManagement         │
// │  → allows/denies mode switch                         │
// │  → publishes vehicle_status (nav_state)              │
// │  → publishes vehicle_control_mode (with overrides)   │
// └────────────┬──────────────────────────────────────┬──┘
//              │                                      │
//              ↓                                      ↓
// ┌─────────────────────────┐        ┌──────────────────────────┐
// │   Mode Executor         │        │  FlightModeManager       │
// │   (Companion/ZTSS)      │        │  (internal modes)        │
// │                         │        │                          │
// │ • Publishes trajectory_ │        │ • Handles AUTO/POSCTL    │
// │   setpoint when active  │        │ • Publishes trajectory_  │
// │ • Sends vehicle_command │        │   setpoint for internal  │
// │   via mode_executor     │        │   modes                  │
// │   topic                 │        │ • Stops publishing if    │
// └────────────┬────────────┘        │   external mode active   │
//              │                     └──────────┬───────────────┘
//              │                                │
//              └────────────────┬───────────────┘
//                               ↓
//                  ┌─────────────────────────────┐
//                  │   Controllers               │
//                  │   (mc_pos_control, etc.)    │
//                  │                             │
//                  │ • Consume trajectory_       │
//                  │   setpoint                  │
//                  │ • Use vehicle_control_mode  │
//                  │   flags to decide behavior  │
//                  └─────────────────────────────┘
