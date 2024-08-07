/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include "redundancy.hpp"
#include <px4_log.h>
#include <drivers/drv_hrt.h>
#include <board_config.h>
#include <lib/parameters/param.h>

using namespace time_literals;

const unsigned REDUNDANCY_INTERVAL_MS = 10_ms;

Redundancy::Redundancy() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

Redundancy::~Redundancy()
{
}

bool Redundancy::send_vehicle_command(uint16_t cmd, float param1 = NAN, float param2 = NAN)
{
	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = NAN;
	vcmd.param4 = NAN;
	vcmd.param5 = (double)NAN;
	vcmd.param6 = (double)NAN;
	vcmd.param7 = NAN;
	vcmd.command = cmd;
	vcmd.target_system = _status[_controller_idx].system_id;
	vcmd.target_component = _status[_controller_idx].component_id;

	// publish the vehicle command
	return _pub_vehicle_command.publish(vcmd);
}

void Redundancy::force_arm()
{
	// 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			     static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
			     21196.f);
}

void Redundancy::force_disarm()
{
	// 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			     static_cast<float>(vehicle_command_s::ARMING_ACTION_DISARM),
			     21196.f);
}

void Redundancy::manage_primary_arming()
{
	static int redundant_controllers_arming;
	const int wait_max = 1_s / REDUNDANCY_INTERVAL_MS;
	bool armed = _status[PRIMARY_FC_IDX].arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (!armed) {
		redundant_controllers_arming = 0;
	}

	/* If spare controllers didn't arm in time. If counter is > wait_max, and spare is not armed, it means that the spare has disarmed (mid flight). */

	if (_status[PRIMARY_FC_IDX + 1].arming_state != vehicle_status_s::ARMING_STATE_ARMED
	    && redundant_controllers_arming == wait_max) {
		PX4_ERR("Spare controller not functional, disarming!");

		force_disarm();
	}

	/* Armed; wait for secondary FCs to arm */

	if (armed && redundant_controllers_arming < wait_max) {
		/* TODO: check all the configured redundant controllers */

		if (_status[PRIMARY_FC_IDX + 1].arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			PX4_INFO("Spare controllers armed OK");

			/* Set the counter to wait_max + 1. This prevents
			 * disarming the primary in case the secondary disarms
			 * during flights (the if clause above).
			 */

			redundant_controllers_arming = wait_max + 1;

		} else {
			redundant_controllers_arming++;
		}
	}
}

void Redundancy::manage_spare_arming()
{
	bool armed = _status[_controller_idx].arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	/* Arm if primary FC is armed */

	if (!armed && _status[PRIMARY_FC_IDX].arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		PX4_INFO("Arming as primary FC armed");

		force_arm();
	}
}

void Redundancy::manage_spare_disarming()
{
	static bool armed = false;
	const int landed_max = 4_s / REDUNDANCY_INTERVAL_MS;
	static int landed = -1;

	/* Auto-disarm in 4 seconds after landing if primary is
	 * disarmed. TODO: Separate parameter for redundant FC auto disarm delay
	 */

	/* Reset counter if we are airborne or we just armed */

	if (!_landed.landed ||
	    (!armed && _status[_controller_idx].arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
		landed = landed_max;
	}

	armed = _status[_controller_idx].arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	/* If we are landed, decrement counter */

	if (_landed.landed && landed > 0) {
		landed--;
	}

	/* We have been landed the needed time, allowed to disarm as soon as primary is disarmed */

	if (landed == 0 && armed &&
	    _status[PRIMARY_FC_IDX].arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		PX4_INFO("Disarming as landed and primary disarmed");
		force_disarm();
		landed = -1;
	}
}

void Redundancy::manage_primary()
{
	manage_primary_arming();
}

void Redundancy::manage_spare()
{
	manage_spare_arming();
	manage_spare_disarming();
}

void Redundancy::Run()
{
	_landed_sub.copy(&_landed);

	for (int i = 0; i < vehicle_status_s::MAX_REDUNDANT_CONTROLLERS; i++) {
		if (i != _controller_idx) {
			// Some other controller
			_redundant_status_sub[i].copy(&_status[i]);

		} else {
			// This controller
			_vehicle_status_sub.copy(&_status[i]);
		}
	}

	if (_controller_idx == PRIMARY_FC_IDX) {
		manage_primary();

	} else {
		manage_spare();
	}

	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	ScheduleDelayed(REDUNDANCY_INTERVAL_MS);
}

int Redundancy::init()
{
	int ret;
	int32_t mav_comp_id;
	ret = param_get(param_find("MAV_COMP_ID"), &mav_comp_id);

	if (ret == PX4_OK && mav_comp_id > 0 && mav_comp_id <= vehicle_status_s::MAX_REDUNDANT_CONTROLLERS) {
		_controller_idx = mav_comp_id - 1; // - MAV_COMP_ID_AUTOPILOT1

		ScheduleNow();

		PX4_INFO("Redundancy module started");

	} else {
		ret = PX4_ERROR;
	}

	return ret;
}

int Redundancy::task_spawn(int argc, char *argv[])
{
	Redundancy *instance = new Redundancy();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		return instance->init();

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int Redundancy::custom_command(int argc, char *argv[])
{
	print_usage("unrecognized command");

	return 0;
}

int Redundancy::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically to perform redundancy related tasks,
such as arming / disarming redundant FCs and doing fault detection and handling

The tasks can be started via CLI
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("redundancy", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int redundancy_main(int argc, char *argv[])
{
	return Redundancy::main(argc, argv);
}
