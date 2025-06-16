/****************************************************************************
 *
 *   Copyright (c) 2025 Technology Innovation Institute. All rights reserved.
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

/**
 * @file critical_action.h
 * System critical action request/release functions definitions
 *
 */

#pragma once

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

const uint8_t ACTION_UNKNOWN_COMP_ID = 0;
const uint8_t ACTION_FWUPDATER_COMP_ID = 1;
const uint8_t ACTION_SENSOR_CALIBRATION_COMP_ID = 2;
const uint8_t ACTION_FLIGHT_LOG_DLOAD_COMP_ID = 3;
const uint8_t ACTION_MAVLINK_FTP_COMP_ID = 4;
const uint8_t ACTION_COMMANDER_COMP_ID = 5;
const uint8_t ACTION_ASSEMBLY_AGENT_COMP_ID = 6;

const size_t ACTION_COUNT = 7;

class CriticalAction
{
public:
	CriticalAction() {};
	~CriticalAction() {};

	bool request(uint8_t comp_id);
	void release(uint8_t comp_id);
	void enable(bool state) { _enabled = state; }

private:
	uORB::Publication<vehicle_command_s>              _crit_act_req_pub{ORB_ID(critical_act_req)};
	uORB::SubscriptionBlocking<vehicle_command_ack_s> _crit_act_resp_sub{ORB_ID(critical_act_resp)};
	bool _enabled{false};
};
