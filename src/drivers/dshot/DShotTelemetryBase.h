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

class DShotTelemetryBase
{
public:
	struct EscData {
		hrt_abstime time;
		int8_t temperature;  ///< [deg C]
		int16_t voltage;     ///< [0.01V]
		int16_t current;     ///< [0.01A]
		int16_t consumption; ///< [mAh]
		int16_t erpm;        ///< [100ERPM]
	};

	static constexpr int esc_info_size_blheli32 = 64;
	static constexpr int esc_info_size_kiss_v1 = 15;
	static constexpr int esc_info_size_kiss_v2 = 21;
	static constexpr int max_esc_info_size = esc_info_size_blheli32;

	struct OutputBuffer {
		uint8_t buffer[max_esc_info_size];
		int buf_pos{0};
		int motor_index;
	};

	virtual ~DShotTelemetryBase() = default;

	virtual int init(const char *device)
	{
		(void)device;
		return 0;
	}

	virtual void deinit() {}

	virtual void setNumMotors(int num_motors)
	{
		(void)num_motors;
	}

	virtual int numMotors() const { return 0; }

	virtual int update() { return -1; }

	virtual int redirectOutput(OutputBuffer &buffer)
	{
		(void)buffer;
		return -1;
	}

	virtual bool redirectActive() const { return false; }

	virtual int getRequestMotorIndex() { return -1; }

	virtual const EscData &latestESCData() const { return _empty_data; }
	virtual bool expectingData() const { return false; }
	virtual int batchSampleCount() const { return 0; }
	virtual int batchSampleIndex(int sample) const
	{
		(void)sample;
		return -1;
	}
	virtual const EscData &batchSampleData(int sample) const
	{
		(void)sample;
		return _empty_data;
	}

	virtual void printStatus() const {}

	virtual void decodeAndPrintEscInfoPacket(const OutputBuffer &buffer)
	{
		(void)buffer;
	}

	virtual bool supportsEscInfoRequest() const { return false; }

	// Optional hook for bidirectional backends mapping telemetry index -> output channel.
	virtual void setMotorOutputMap(int telemetry_index, uint8_t output_channel)
	{
		(void)telemetry_index;
		(void)output_channel;
	}

private:
	EscData _empty_data{};
};
