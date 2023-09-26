/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "log_writer_mavlink.h"
#include "messages.h"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <cstring>

namespace px4
{
namespace logger
{

LogWriterMavlink::LogWriterMavlink()
{
	_ulog_stream_data.length = 0;
	_ulog_stream_acked_data.length = 0;
}

bool LogWriterMavlink::init()
{
	return true;
}

LogWriterMavlink::~LogWriterMavlink()
{
	if (orb_sub_valid(_ulog_stream_ack_sub)) {
		orb_unsubscribe(_ulog_stream_ack_sub);
	}
}

void LogWriterMavlink::start_log()
{
	if (!orb_sub_valid(_ulog_stream_ack_sub)) {
		_ulog_stream_ack_sub = orb_subscribe(ORB_ID(ulog_stream_ack));
	}

	// make sure we don't get any stale ack's by doing an orb_copy
	ulog_stream_ack_s ack;
	orb_copy(ORB_ID(ulog_stream_ack), _ulog_stream_ack_sub, &ack);

	_ulog_stream_data.msg_sequence = 0;
	_ulog_stream_data.length = 0;
	_ulog_stream_data.first_message_offset = 0;

	_ulog_stream_acked_data.msg_sequence = 0;
	_ulog_stream_acked_data.length = 0;
	_ulog_stream_acked_data.first_message_offset = 0;

	_is_started = true;
}

void LogWriterMavlink::stop_log()
{
	_ulog_stream_data.length = 0;
	_ulog_stream_acked_data.length = 0;
	_is_started = false;
}

int LogWriterMavlink::write_message(void *ptr, size_t size, bool acked)
{
	if (!is_started()) {
		return 0;
	}

	ulog_stream_s *ulog_s_p;

	if (acked) {
		ulog_s_p = &_ulog_stream_acked_data;

	} else {
		ulog_s_p = &_ulog_stream_data;
	}

	const uint8_t data_len = (uint8_t)sizeof(ulog_s_p->data);
	uint8_t *ptr_data = (uint8_t *)ptr;

	if (ulog_s_p->first_message_offset == 255) {
		ulog_s_p->first_message_offset = ulog_s_p->length;
	}

	while (size > 0) {
		size_t send_len = math::min((size_t)data_len - ulog_s_p->length, size);
		memcpy(ulog_s_p->data + ulog_s_p->length, ptr_data, send_len);
		ulog_s_p->length += send_len;
		ptr_data += send_len;
		size -= send_len;

		if (ulog_s_p->length >= data_len) {
			if (publish_message(acked)) {
				return -2;
			}
		}
	}

	return 0;
}

void LogWriterMavlink::set_need_reliable_transfer(bool need_reliable)
{
#ifndef LOGGER_PARALLEL_LOGGING

	if (!need_reliable && _need_reliable_transfer) {
		if (_ulog_stream_data.length > 0) {
			// make sure to send previous data using reliable transfer
			publish_message();
		}
	}

	_need_reliable_transfer = need_reliable;
#endif
}

int LogWriterMavlink::publish_message(bool acked)
{
	ulog_stream_s *ulog_s_p;

	if (acked) {
		ulog_s_p = &_ulog_stream_acked_data;

	} else {
		ulog_s_p = &_ulog_stream_data;
	}

	ulog_s_p->timestamp = hrt_absolute_time();
	ulog_s_p->flags = 0;

#ifdef LOGGER_PARALLEL_LOGGING

	if (!acked) {
		_ulog_stream_pub.publish(*ulog_s_p);

	} else {
		ulog_s_p->flags = ulog_s_p->FLAGS_NEED_ACK;
		_ulog_stream_acked_pub.publish(*ulog_s_p);
#else

	if (_need_reliable_transfer) {
		_ulog_stream_data.flags = _ulog_stream_data.FLAGS_NEED_ACK;
	}

	_ulog_stream_pub.publish(_ulog_stream_data);

	if (_need_reliable_transfer) {
#endif
		// we need to wait for an ack. Note that this blocks the main logger thread, so if a file logging
		// is already running, it will miss samples.
		px4_pollfd_struct_t fds[1];
		fds[0].fd = _ulog_stream_ack_sub;
		fds[0].events = POLLIN;
		bool got_ack = false;
		const int timeout_ms = ulog_stream_ack_s::ACK_TIMEOUT * ulog_stream_ack_s::ACK_MAX_TRIES;

		hrt_abstime started = hrt_absolute_time();

		do {
			int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), timeout_ms);

			if (ret <= 0) {
				break;
			}

			if (fds[0].revents & POLLIN) {
				ulog_stream_ack_s ack;
				orb_copy(ORB_ID(ulog_stream_ack), _ulog_stream_ack_sub, &ack);

				if (ack.msg_sequence == ulog_s_p->msg_sequence) {
					got_ack = true;
				}

			} else {
				break;
			}
		} while (!got_ack && hrt_elapsed_time(&started) / 1000 < timeout_ms);

		if (!got_ack) {
			PX4_ERR("Ack timeout. Stopping mavlink log");
			stop_log();
			return -2;
		}

		PX4_DEBUG("got ack in %i ms", (int)(hrt_elapsed_time(&started) / 1000));
	}

	ulog_s_p->msg_sequence++;
	ulog_s_p->length = 0;
	ulog_s_p->first_message_offset = 255;
	return 0;
}

}
}
