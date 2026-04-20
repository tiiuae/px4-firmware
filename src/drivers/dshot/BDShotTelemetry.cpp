/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "BDShotTelemetry.h"

#include <drivers/drv_dshot.h>
#include <px4_platform_common/log.h>

#include <errno.h>

using namespace time_literals;

int BDShotTelemetry::init(const char *unused_device)
{
	(void)unused_device;
	return 0;
}

void BDShotTelemetry::setNumMotors(int num_motors)
{
	if (num_motors < 0) {
		_num_motors = 0;

	} else if (num_motors > MAX_ESC_COUNT) {
		_num_motors = MAX_ESC_COUNT;

	} else {
		_num_motors = num_motors;
	}
}

void BDShotTelemetry::setMotorOutputMap(int telemetry_index, uint8_t output_channel)
{
	if (telemetry_index >= 0 && telemetry_index < MAX_ESC_COUNT) {
		_motor_output_map[telemetry_index] = output_channel;
	}
}

int BDShotTelemetry::update()
{
	if (_num_motors <= 0) {
		return -1;
	}

	const uint16_t ready_mask = up_bdshot_get_ready_mask();
	return (ready_mask != 0) ? 0 : -1;
}

int BDShotTelemetry::redirectOutput(OutputBuffer &buffer)
{
	(void)buffer;
	return -EOPNOTSUPP;
}

int BDShotTelemetry::getRequestMotorIndex()
{
	if (_num_motors <= 0) {
		return -1;
	}

	return 0;
}

void BDShotTelemetry::printStatus() const
{
	PX4_INFO("BDShot telemetry ready");
}

void BDShotTelemetry::decodeAndPrintEscInfoPacket(const OutputBuffer &buffer)
{
	(void)buffer;
	PX4_WARN("ESC info request is not supported with bidirectional DShot telemetry");
}
