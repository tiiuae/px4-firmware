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

#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/topics/esc_status.h>

#include "DShotTelemetryBase.h"

class BDShotTelemetry : public DShotTelemetryBase
{
public:
	static constexpr int MAX_ESC_COUNT = 16;

	int init(const char *unused_device);
	void deinit() {}

	void setNumMotors(int num_motors);
	int numMotors() const { return _num_motors; }

	void setMotorOutputMap(int telemetry_index, uint8_t output_channel);

	/**
	 * Read telemetry from bidirectional dshot wrapper and handle timeouts.
	 * @return -1 if no update, -2 timeout, >= 0 for the telemetry index.
	 */
	int update();

	int redirectOutput(OutputBuffer &buffer);

	bool redirectActive() const { return false; }

	int getRequestMotorIndex();

	const EscData &latestESCData() const { return _latest_data; }

	bool expectingData() const { return false; }

	void printStatus() const;

	void decodeAndPrintEscInfoPacket(const OutputBuffer &buffer);

private:
	EscData _latest_data{};
	int _num_motors{0};
	uint8_t _motor_output_map[MAX_ESC_COUNT] {};
};
