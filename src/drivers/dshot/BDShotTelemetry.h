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

#include <drivers/drv_dshot.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/esc_status.h>

#include "DShotTelemetry.h"

class BDShotTelemetry
{
public:
	BDShotTelemetry();

	int init(const char *serial_device);

	void setNumMotors(int num_motors);
	int numMotors() const { return _num_motors; }
	void setActuatorFunction(int telemetry_index, uint8_t function);

	int getRequestMotorIndex();
	bool expectingData() const;

	void update(uint32_t output_mask, const int8_t *output_to_telemetry_index,
		    unsigned num_outputs, bool bidirectional_enabled, int pole_count);

	void printStatus() const;

	DShotTelemetry &serial() { return _serial; }

private:
	void handle_serial_telemetry_data(int telemetry_index, const DShotTelemetry::EscData &data, int pole_count);
	void handle_bdshot_telemetry(uint32_t output_mask, const int8_t *output_to_telemetry_index,
				     unsigned num_outputs, int pole_count);
	void publish(bool clear_samples);

	DShotTelemetry _serial{};
	uORB::PublicationMultiData<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	esc_status_s _esc_status{};

	int _num_motors{0};
	int _last_telemetry_index{-1};
	uint8_t _actuator_functions[esc_status_s::CONNECTED_ESC_MAX] {};
};
