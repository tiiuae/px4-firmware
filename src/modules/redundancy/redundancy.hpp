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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>

#define PRIMARY_FC_IDX 0 //MAV_COMP_ID_AUTOPILOT1 - 1

class Redundancy final : public ModuleBase<Redundancy>, public px4::ScheduledWorkItem
{
public:
	Redundancy();
	~Redundancy() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

private:
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _redundant_status_sub[vehicle_status_s::MAX_REDUNDANT_CONTROLLERS] {ORB_ID(redundant_status0), ORB_ID(redundant_status1)};
	vehicle_status_s _status[vehicle_status_s::MAX_REDUNDANT_CONTROLLERS];

	uORB::Subscription _landed_sub{ORB_ID(vehicle_land_detected)};
	vehicle_land_detected_s _landed;

	uORB::Publication<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};

	bool _autopilot_timeout[vehicle_status_s::MAX_REDUNDANT_CONTROLLERS] {};
	int _n_autopilots;
	int _controller_idx;
	int _auto_disarm_min_time;

	int init(int spare_autopilots);

	bool send_vehicle_command(uint16_t cmd, float param1, float param2);

	void force_arm();
	void force_disarm();

	void manage_primary();
	void manage_spare();
	void manage_primary_arming();
	void manage_spare_arming();
	void manage_spare_disarming();
};
