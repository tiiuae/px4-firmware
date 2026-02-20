/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef SERVO_OUTPUT_RAW_HPP
#define SERVO_OUTPUT_RAW_HPP

#include <uORB/topics/actuator_outputs.h>

template <int N>
class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamServoOutputRaw<N>(mavlink); }

	static constexpr const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "SERVO_OUTPUT_RAW_0";

		case 1:
			return "SERVO_OUTPUT_RAW_1";

		case 2:
			return "SERVO_OUTPUT_RAW_2";

		case 3:
			return "SERVO_OUTPUT_RAW_3";
		}

		return "SERVO_OUTPUT_RAW";
	}

	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _act_sub.advertised() ? MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _act_sub{ORB_ID(actuator_outputs), N};

	bool send() override
	{
		actuator_outputs_s act;

		if (_act_sub.update(&act)) {
			mavlink_servo_output_raw_t msg;
			unsigned i;

			static_assert(sizeof(act.output) / sizeof(act.output[0]) >= 16, "mavlink message requires at least 16 outputs");

			/* Initally, fill all _raw values with 0xff (invalid) */

			memset(&msg, 0xff, sizeof(msg));

			msg.time_usec = act.timestamp;
			msg.port = N;

			/* Assume msg is packed without filler bytes between servo values */
			static_assert((&msg.servo16_raw - &msg.servo9_raw + 1) == 8);
			static_assert((&msg.servo8_raw - &msg.servo1_raw + 1) == 8);

			for (i = 0; i < act.noutputs && i < 8; i++) {
				(&msg.servo1_raw)[i] = act.output[i];
			}

			for (; i < act.noutputs && i < 16; i++) {
				(&msg.servo9_raw)[i - 8] = act.output[i];
			}

			mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // SERVO_OUTPUT_RAW_HPP
