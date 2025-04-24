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
 * @file critical_action.cpp
 * Commander critical action request/release functions implementation
 *
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>


const uint64_t CRIT_ACT_RESP_TIMEOUT = 1000000;


const uint8_t  CRIT_ACT_SRC_COMP_ID = 5;
const uint32_t CRIT_ACT_CMD_REQUEST = vehicle_command_s::VEHICLE_CMD_CUSTOM_1;
const uint32_t CRIT_ACT_CMD_RELEASE = vehicle_command_s::VEHICLE_CMD_CUSTOM_2;

static vehicle_command_s critical_req {};
static orb_advert_t crit_act_req_pub = ORB_ADVERT_INVALID;
static orb_sub_t    crit_act_resp_sub = ORB_SUB_INVALID;

int critical_action_init()
{
	crit_act_req_pub = orb_advertise(ORB_ID(critical_act_req), &critical_req);
	crit_act_resp_sub = orb_subscribe(ORB_ID(critical_act_resp));

	return PX4_OK;
}

bool critical_action_request()
{
	if (!orb_advert_valid(crit_act_req_pub) || !orb_sub_valid(crit_act_resp_sub)) {
		PX4_ERR("Uninitialized CriticalAction uorb handles");
		return false;
	}

	PX4_INFO("Request CriticalAction");
	bool critical_allowed = false;
	bool response_received = false;
	hrt_abstime now = hrt_absolute_time();

	critical_req.timestamp = now;
	critical_req.command = CRIT_ACT_CMD_REQUEST;
	critical_req.source_component = CRIT_ACT_SRC_COMP_ID;

	if (PX4_OK != orb_publish(ORB_ID(critical_act_req), &crit_act_req_pub, &critical_req)) {
		PX4_ERR("Failed to publish CriticalAction uorb request");
		return false;
	}

	while (!response_received && hrt_elapsed_time(&now) < CRIT_ACT_RESP_TIMEOUT) {

		bool updated = false;
		orb_check(crit_act_resp_sub, &updated);

		if (updated) {
			vehicle_command_ack_s critical_resp{};
			orb_copy(ORB_ID(critical_act_resp), crit_act_resp_sub, &critical_resp);

			switch (critical_resp.command) {
			case vehicle_command_s::VEHICLE_CMD_CUSTOM_1: {
					response_received = true;

					if (critical_resp.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED)	{
						critical_allowed = true;
						PX4_INFO("CriticalAction - ALLOWED");

					} else if (critical_resp.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) {
						critical_allowed = false;
						PX4_INFO("CriticalAction - DENIED");

					} else {
						PX4_WARN("CriticalAction - UKNOWN RESPONSE! %d", critical_resp.result);
					}
				}
				break;

			default:
				break;
			}
		}

		if (!response_received) {
			px4_usleep(1000);
		}
	}

	return critical_allowed;
}

void critical_action_release()
{
	if (!orb_advert_valid(crit_act_req_pub)) {
		PX4_ERR("Uninitialized Critical Action uorb handles");
		return;
	}

	PX4_INFO("Release Critical Action");
	critical_req.timestamp = hrt_absolute_time();
	critical_req.command = CRIT_ACT_CMD_RELEASE;
	critical_req.source_component = CRIT_ACT_SRC_COMP_ID;
	if (PX4_OK != orb_publish(ORB_ID(critical_act_req), &crit_act_req_pub, &critical_req)) {
		PX4_ERR("Failed to publish CriticalAction uorb request");
	}
}


