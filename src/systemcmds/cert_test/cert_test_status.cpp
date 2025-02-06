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

#include <sys/stat.h>
#include <fcntl.h>
#include "test_logger.hpp"

CertTestStatus::CertTestStatus(uint32_t hw, BgProcExec *actuator, CanTest *can_test, TestLogger *log, bool verbose) :
	_saluki_hw(hw),
	_actuator(actuator),
	_can_test(can_test),
	_log(log),
	_verbose(verbose)
{
	_status_pub.advertise();
}

void CertTestStatus::update()
{
	_cert_status.timestamp = hrt_absolute_time();

	bool log_px4io = false;

	for (int i = 0; _orb_report[i].orb != nullptr; i++) {
		const struct orb_report_wrapper *report = &_orb_report[i];

		if (report->saluki_hw != SALUKI_HW_ANY &&
			report->saluki_hw != _saluki_hw) {
			continue;
		}

		report->orb->get_latest();
		report->status[LATEST] = report->orb->get_result();
		report->status[ERR_RECORD] |= (report->status[LATEST] & ~(OrbBase::STATUS_OK));

		if (report->status == _cert_status.px4io_status) {
			log_px4io = true;
		}
	}

	// set error record also for telem test already recovered errors
	if (_telem_test.error_record()) {
		_cert_status.telem_test[ERR_RECORD] |= OrbBase::STATUS_TEST_FAIL;
	}

	int pid = _actuator->get_pid(true);
	if (pid < 0) {
		_cert_status.actuator_test[LATEST] = OrbBase::STATUS_INIT | OrbBase::STATUS_NOT_RUNNING;
		_cert_status.actuator_test[ERR_RECORD] |= (_cert_status.actuator_test[LATEST] & ~(OrbBase::STATUS_OK));
	} else {
		_cert_status.actuator_test[LATEST] = OrbBase::STATUS_OK;
	}

	_cert_status.cansend[LATEST] = _can_test->status;
	_cert_status.cansend[ERR_RECORD] |= (_cert_status.cansend[LATEST] & ~(OrbBase::STATUS_OK));

	_status_pub.publish(_cert_status);

	if (_sent_time == 0 ||
		hrt_absolute_time() - _sent_time > REPORT_INTERVAL) {

		struct timespec ts = {0};
		abstime_to_ts(&ts, hrt_absolute_time());
		PX4_INFO("");
		_log->log(_log->INFO, "status (uptime %ds)", ts.tv_sec);
		_log->log(_log->INFO, "    mag:        0x%x (0x%x)", _cert_status.sensor_mag[0], _cert_status.sensor_mag[1]);
		_log->log(_log->INFO, "    baro:       0x%x (0x%x)", _cert_status.sensor_baro[0], _cert_status.sensor_baro[1]);
		_log->log(_log->INFO, "    accel:      0x%x (0x%x)", _cert_status.sensor_accel[0], _cert_status.sensor_accel[1]);
		_log->log(_log->INFO, "    gyro:       0x%x (0x%x)", _cert_status.sensor_gyro[0], _cert_status.sensor_gyro[1]);
		_log->log(_log->INFO, "    airspeed:   0x%x (0x%x)", _cert_status.sensor_airspeed[0], _cert_status.sensor_airspeed[1]);
		_log->log(_log->INFO, "    adc:        0x%x (0x%x)", _cert_status.adc_report[0], _cert_status.adc_report[1]);
		_log->log(_log->INFO, "    uart:       0x%x (0x%x)", _cert_status.telem_test[0], _cert_status.telem_test[1]);
		_log->log(_log->INFO, "    actuator:   0x%x (0x%x)", _cert_status.actuator_test[0], _cert_status.actuator_test[1]);
		_log->log(_log->INFO, "    cansend:    0x%x (0x%x)", _cert_status.cansend[0], _cert_status.cansend[1]);
		if (log_px4io) {
			_log->log(_log->INFO, "    px4io:      0x%x (0x%x)", _cert_status.px4io_status[0], _cert_status.px4io_status[1]);
		}

		_sent_time = hrt_absolute_time();
	}
}
