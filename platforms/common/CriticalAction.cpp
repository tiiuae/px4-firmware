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

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

const uint64_t CRIT_ACT_RESP_TIMEOUT = 1000000;
const uint32_t CRIT_ACT_CMD_REQUEST = vehicle_command_s::VEHICLE_CMD_CUSTOM_1;
const uint32_t CRIT_ACT_CMD_RELEASE = vehicle_command_s::VEHICLE_CMD_CUSTOM_2;

CriticalAction::CriticalAction()
{
	_crit_act_req_pub = orb_advertise(ORB_ID(critical_act_req), &_critical_req);
	_crit_act_resp_sub = orb_subscribe(ORB_ID(critical_act_resp));
}

CriticalAction::~CriticalAction()
{
	if (orb_advert_valid(_crit_act_req_pub)) {
		orb_unadvertise(_crit_act_req_pub);
	}
	if (orb_sub_valid(_crit_act_resp_sub)) {
		orb_unsubscribe(_crit_act_resp_sub);
	}
}

bool CriticalAction::request(uint8_t comp_id)
{
        if (!orb_advert_valid(_crit_act_req_pub) || !orb_sub_valid(_crit_act_resp_sub)) {
            px4_log_raw(_PX4_LOG_LEVEL_ERROR, "Uninitialized CriticalAction uorb handles\n");
            return false;
        }

        // Allow activity in case MOI-agent does not respond for some reason.
        bool critical_allowed = true;
        bool response_received = false;
        hrt_abstime now = hrt_absolute_time();

        _critical_req.timestamp = now;
        _critical_req.command = CRIT_ACT_CMD_REQUEST;
        _critical_req.source_component = comp_id;

        if (PX4_OK != orb_publish(ORB_ID(critical_act_req), &_crit_act_req_pub, &_critical_req)) {
            px4_log_raw(_PX4_LOG_LEVEL_ERROR, "Failed to publish CriticalAction uorb request\n");
            return false;
        }

        while (!response_received && hrt_elapsed_time(&now) < CRIT_ACT_RESP_TIMEOUT) {

            bool updated = false;
            orb_check(_crit_act_resp_sub, &updated);

            if (updated) {
                vehicle_command_ack_s critical_resp{};
                orb_copy(ORB_ID(critical_act_resp), _crit_act_resp_sub, &critical_resp);

                switch (critical_resp.command) {
                case vehicle_command_s::VEHICLE_CMD_CUSTOM_1: {
                        response_received = true;

                        if (critical_resp.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED)	{
                            critical_allowed = true;

                        } else if (critical_resp.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) {
                            critical_allowed = false;
                            px4_log_raw(_PX4_LOG_LEVEL_WARN, "CriticalAction - DENIED\n");

                        } else {
                            px4_log_raw(_PX4_LOG_LEVEL_WARN, "CriticalAction - UNKNOWN RESPONSE! %d\n", critical_resp.result);
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

void CriticalAction::release(uint8_t comp_id)
{
        if (!orb_advert_valid(_crit_act_req_pub)) {
            px4_log_raw(_PX4_LOG_LEVEL_ERROR, "Uninitialized Critical Action uorb handles\n");
            return;
        }

        _critical_req.timestamp = hrt_absolute_time();
        _critical_req.command = CRIT_ACT_CMD_RELEASE;
        _critical_req.source_component = comp_id;
        if (PX4_OK != orb_publish(ORB_ID(critical_act_req), &_crit_act_req_pub, &_critical_req)) {
            px4_log_raw(_PX4_LOG_LEVEL_ERROR, "Failed to publish CriticalAction uorb request\n");
        }
}
