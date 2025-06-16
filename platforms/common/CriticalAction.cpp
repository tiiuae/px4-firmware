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
 * @file CriticalAction.cpp
 * System critical action request/release functions implementation
 *
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/critical_action.h>

const uint64_t CRIT_ACT_RESP_TIMEOUT_US = 1'000'000; // 1s
const uint32_t CRIT_ACT_CMD_REQUEST = vehicle_command_s::VEHICLE_CMD_CUSTOM_1;
const uint32_t CRIT_ACT_CMD_RELEASE = vehicle_command_s::VEHICLE_CMD_CUSTOM_2;

bool CriticalAction::request(uint8_t comp_id)
{
	if (!_enabled) {
		return true;
	}

	// Send request

	vehicle_command_s critical_req{};
	hrt_abstime now = hrt_absolute_time();

	critical_req.timestamp = now;
	critical_req.command = CRIT_ACT_CMD_REQUEST;
	critical_req.source_component = (uint16_t)comp_id;

	if (!_crit_act_req_pub.publish(critical_req)) {
		px4_log_raw(_PX4_LOG_LEVEL_ERROR, "ERROR [CriticalAction} Failed to publish uorb request\n");
		return false;
	}

	// Wait for response...

	bool critical_allowed {true};
	bool response_received {false};
	vehicle_command_ack_s critical_resp{};

	while (!response_received && hrt_elapsed_time(&now) < CRIT_ACT_RESP_TIMEOUT_US) {

		if (_crit_act_resp_sub.updateBlocking(critical_resp, CRIT_ACT_RESP_TIMEOUT_US)) {
			switch (critical_resp.command) {
			case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
				if (critical_resp.target_component == critical_req.source_component) {
					response_received = true;

					if (critical_resp.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED)	{
						critical_allowed = true;

					} else if (critical_resp.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) {
						critical_allowed = false;
						px4_log_raw(_PX4_LOG_LEVEL_WARN, "WARN  [CriticalAction] DENIED\n");

					} else {
						px4_log_raw(_PX4_LOG_LEVEL_ERROR, "ERROR [CriticalAction] Unknown result value: %d\n", critical_resp.result);
					}
				}

				break;

			default:
				break;
			}
		}

	}

	if (!response_received) {
		px4_log_raw(_PX4_LOG_LEVEL_WARN, "WARN  [CriticalAction] Response wait timeout \n");
	}

	return critical_allowed;
}

void CriticalAction::release(uint8_t comp_id)
{
	if (!_enabled) {
		return;
	}

	vehicle_command_s critical_req{};
	critical_req.timestamp = hrt_absolute_time();
	critical_req.command = CRIT_ACT_CMD_RELEASE;
	critical_req.source_component = (uint16_t)comp_id;

	if (!_crit_act_req_pub.publish(critical_req)) {
		px4_log_raw(_PX4_LOG_LEVEL_ERROR, "ERROR [CriticalAction] Failed to publish uorb request\n");
	}
}
