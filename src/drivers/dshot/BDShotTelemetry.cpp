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

#include <px4_platform_common/px4_config.h>

#include <mathlib/mathlib.h>

#include <cstring>

BDShotTelemetry::BDShotTelemetry()
{
	_esc_status_pub.advertise();
}

int BDShotTelemetry::init(const char *serial_device)
{
	return _serial.init(serial_device);
}

void BDShotTelemetry::setNumMotors(int num_motors)
{
	_num_motors = math::min(num_motors, (int)esc_status_s::CONNECTED_ESC_MAX);
}

void BDShotTelemetry::setActuatorFunction(int telemetry_index, uint8_t function)
{
	if (telemetry_index >= 0 && telemetry_index < esc_status_s::CONNECTED_ESC_MAX) {
		_actuator_functions[telemetry_index] = function;
	}
}

int BDShotTelemetry::getRequestMotorIndex()
{
	return _serial.getRequestMotorIndex();
}

bool BDShotTelemetry::expectingData() const
{
	return _serial.expectingData();
}

void BDShotTelemetry::update(uint32_t output_mask, const int8_t *output_to_telemetry_index,
			     unsigned num_outputs, bool bidirectional_enabled, int pole_count)
{
	int serial_update = _serial.update();

	if (serial_update >= 0) {
		handle_serial_telemetry_data(serial_update, _serial.latestESCData(), pole_count);
	}

	if (bidirectional_enabled) {
		handle_bdshot_telemetry(output_mask, output_to_telemetry_index, num_outputs, pole_count);
	}
}

void BDShotTelemetry::handle_serial_telemetry_data(int telemetry_index, const DShotTelemetry::EscData &data,
		int pole_count)
{
	if (telemetry_index >= 0 && telemetry_index < esc_status_s::CONNECTED_ESC_MAX) {
		_esc_status.esc_online_flags |= 1 << telemetry_index;
		_esc_status.esc_armed_flags |= 1 << telemetry_index;

		_esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];
		_esc_status.esc[telemetry_index].timestamp = data.time;

		const int half_pole_count = math::max(pole_count / 2, 1);
		_esc_status.esc[telemetry_index].esc_rpm = (static_cast<int>(data.erpm) * 100) / half_pole_count;
		_esc_status.esc[telemetry_index].esc_voltage = static_cast<float>(data.voltage) * 0.01f;
		_esc_status.esc[telemetry_index].esc_current = static_cast<float>(data.current) * 0.01f;
		_esc_status.esc[telemetry_index].esc_temperature = static_cast<float>(data.temperature);
	}

	if (telemetry_index <= _last_telemetry_index) {
		publish(true);
	}

	_last_telemetry_index = telemetry_index;
}

void BDShotTelemetry::handle_bdshot_telemetry(uint32_t output_mask, const int8_t *output_to_telemetry_index,
		unsigned num_outputs, int pole_count)
{
	const uint16_t ready_mask = up_bdshot_get_ready_mask();

	if (ready_mask == 0) {
		return;
	}

	bool any_update = false;
	const int half_pole_count = math::max(pole_count / 2, 1);

	for (unsigned output_channel = 0; output_channel < num_outputs; output_channel++) {
		if ((output_mask & (1u << output_channel)) == 0 || (ready_mask & (1u << output_channel)) == 0) {
			continue;
		}

		const int telemetry_index = output_to_telemetry_index[output_channel];

		if (telemetry_index < 0 || telemetry_index >= esc_status_s::CONNECTED_ESC_MAX) {
			continue;
		}

		if (up_bdshot_channel_online(output_channel) > 0) {
			int erpm = 0;

			if (up_bdshot_get_erpm(output_channel, &erpm) == PX4_OK) {
				_esc_status.esc[telemetry_index].timestamp = hrt_absolute_time();
				_esc_status.esc[telemetry_index].esc_rpm = (erpm * 100) / half_pole_count;
				_esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];
				_esc_status.esc_online_flags |= 1 << telemetry_index;
				_esc_status.esc_armed_flags |= 1 << telemetry_index;
				any_update = true;
			}
		}

		_esc_status.esc[telemetry_index].esc_errorcount = up_bdshot_num_errors(output_channel);
	}

	if (any_update) {
		publish(false);
	}
}

void BDShotTelemetry::publish(bool clear_samples)
{
	_esc_status.timestamp = hrt_absolute_time();
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
	_esc_status.esc_count = _num_motors;
	++_esc_status.counter;
	_esc_status_pub.update(_esc_status);

	if (clear_samples) {
		memset(&_esc_status.esc, 0, sizeof(_esc_status.esc));
		_esc_status.esc_online_flags = 0;
		_esc_status.esc_armed_flags = 0;
	}
}

void BDShotTelemetry::printStatus() const
{
	_serial.printStatus();
	up_bdshot_status();
}
