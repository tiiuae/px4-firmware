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

#ifndef CRITICAL_ACTION_H_
#define CRITICAL_ACTION_H_

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>

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
    CriticalAction();
    ~CriticalAction();

    bool request(uint8_t comp_id);
    void release(uint8_t comp_id);

private:
    vehicle_command_s _critical_req {};
    orb_advert_t _crit_act_req_pub {ORB_ADVERT_INVALID};
    orb_sub_t    _crit_act_resp_sub {ORB_SUB_INVALID};
};

#endif // CRITICAL_ACTION_H_
