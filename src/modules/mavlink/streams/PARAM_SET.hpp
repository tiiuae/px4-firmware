/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef PARAM_SET_HPP
#define PARAM_SET_HPP

#include <uORB/topics/param_set.h>

class MavlinkStreamParamSet : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamParamSet(mavlink); }

	static constexpr const char *get_name_static() { return "PARAM_SET"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_PARAM_SET; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _request_sub.advertised() ? MAVLINK_MSG_ID_PARAM_SET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamParamSet(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _request_sub{ORB_ID(param_set)};

	bool send() override
	{
		param_set_s request;

		if (_request_sub.update(&request)) {
			mavlink_param_set_t msg{};

			// Set parameter type and value

			switch (request.param_type) {
			case PARAM_TYPE_INT32:
				msg.param_type = MAV_PARAM_TYPE_INT32;
				break;

			case PARAM_TYPE_FLOAT:
				msg.param_type = MAV_PARAM_TYPE_REAL32;
				break;

			default:
				return false;
			}

			msg.param_value = request.value;

			// Set target system/component from uORB message
			msg.target_system = request.target_sysid;
			msg.target_component = request.target_compid;

			// Copy parameter name
			strncpy(msg.param_id, request.parameter_name, sizeof(msg.param_id));

			mavlink_msg_param_set_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // PARAM_SET_HPP
