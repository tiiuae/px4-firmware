/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
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

#include "cert_test_status.hpp"


CertTestStatus::CertTestStatus(BgProcExec *actuator, uint32_t &cansend_status, bool verbose) :
	_actuator(actuator),
	_cansend_status(cansend_status),
	_verbose(verbose)
{
	_status_pub.advertise();
}

void CertTestStatus::update()
{
	_report.timestamp = hrt_absolute_time();

	_mag_report.get_latest();
	_report.sensor_mag[LATEST] = _mag_report.get_result();
	_report.sensor_mag[ERR_RECORD] |= (_report.sensor_mag[LATEST] & ~(OrbBase::STATUS_OK));

	_baro_report.get_latest();
	_report.sensor_baro[LATEST] = _baro_report.get_result();
	_report.sensor_baro[ERR_RECORD] |= (_report.sensor_baro[LATEST] & ~(OrbBase::STATUS_OK));

	_accel_report.get_latest();
	_report.sensor_accel[LATEST] = _accel_report.get_result();
	_report.sensor_accel[ERR_RECORD] |= (_report.sensor_accel[LATEST] & ~(OrbBase::STATUS_OK));

	_gyro_report.get_latest();
	_report.sensor_gyro[LATEST] = _gyro_report.get_result();
	_report.sensor_gyro[ERR_RECORD] |= (_report.sensor_gyro[LATEST] & ~(OrbBase::STATUS_OK));

	_airspeed_report.get_latest();
	_report.sensor_airspeed[LATEST] = _airspeed_report.get_result();
	_report.sensor_airspeed[ERR_RECORD] |= (_report.sensor_airspeed[LATEST] & ~(OrbBase::STATUS_OK));

	_adc_report.get_latest();
	_report.adc_report[LATEST] = _adc_report.get_result();
	_report.adc_report[ERR_RECORD] |= (_report.adc_report[LATEST] & ~(OrbBase::STATUS_OK));

	_telem_test.get_latest();
	_report.telem_test[LATEST] = _telem_test.get_result();
	_report.telem_test[ERR_RECORD] |= (_report.telem_test[LATEST] & ~(OrbBase::STATUS_OK));

	int pid = _actuator->get_pid(true);
	if (pid < 0) {
		_report.actuator_test[LATEST] = OrbBase::STATUS_INIT | OrbBase::STATUS_NOT_RUNNING;
		_report.actuator_test[ERR_RECORD] |= (_report.actuator_test[LATEST] & ~(OrbBase::STATUS_OK));
	} else {
		_report.actuator_test[LATEST] = OrbBase::STATUS_OK;
	}

	_report.cansend[LATEST] = _cansend_status;
	_report.cansend[ERR_RECORD] |= (_report.cansend[LATEST] & ~(OrbBase::STATUS_OK));

	_status_pub.publish(_report);

	if (_sent_time == 0 ||
		hrt_absolute_time() - _sent_time > REPORT_INTERVAL) {

		struct timespec ts = {0};
		abstime_to_ts(&ts, hrt_absolute_time());
		PX4_INFO("");
		PX4_INFO("status (uptime %ds)", ts.tv_sec);
		PX4_INFO("    mag:        0x%x (0x%x)", _report.sensor_mag[0], _report.sensor_mag[1]);
		PX4_INFO("    baro:       0x%x (0x%x)", _report.sensor_baro[0], _report.sensor_baro[1]);
		PX4_INFO("    accel:      0x%x (0x%x)", _report.sensor_accel[0], _report.sensor_accel[1]);
		PX4_INFO("    gyro:       0x%x (0x%x)", _report.sensor_gyro[0], _report.sensor_gyro[1]);
		PX4_INFO("    airspeed:   0x%x (0x%x)", _report.sensor_airspeed[0], _report.sensor_airspeed[1]);
		PX4_INFO("    adc:        0x%x (0x%x)", _report.adc_report[0], _report.adc_report[1]);
		PX4_INFO("    uart:       0x%x (0x%x)", _report.telem_test[0], _report.telem_test[1]);
		PX4_INFO("    actuator:   0x%x (0x%x)", _report.actuator_test[0], _report.actuator_test[1]);
		PX4_INFO("    cansend:    0x%x (0x%x)", _report.cansend[0], _report.cansend[1]);

		_sent_time = hrt_absolute_time();
	}
}