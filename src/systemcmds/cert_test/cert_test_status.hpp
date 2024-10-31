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

#pragma once

#include <px4_platform_common/defines.h>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

#include <uORB/topics/cert_test_status.h>

#include <uORB/topics/telem_test_status.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/airspeed.h>

#include "orb_report.hpp"
#include "bg_proc.hpp"
#include "test_logger.hpp"

using namespace time_literals;

class CertTestStatus
{
public:
	CertTestStatus(BgProcExec *actuator, uint32_t &cansend_status, TestLogger *log, bool verbose);

	~CertTestStatus() = default;

	void update();

private:
	static constexpr hrt_abstime REPORT_INTERVAL = 10_s;

	static constexpr hrt_abstime LISTENER_TIMEOUT = 3_s;
	static constexpr int MAG_INSTANCES = 1;
	static constexpr int BARO_INSTANCES = 2;
	static constexpr int IMU_INSTANCES = 3;
	static constexpr int AIRSPEED_INSTANCES = 1;
	static constexpr int ADC_INSTANCES = 1;
	static constexpr int TELEM_INSTANCES = 1;

	static constexpr int LATEST 		= 0;
	static constexpr int ERR_RECORD 	= 1;

	BgProcExec *_actuator;
	uint32_t &_cansend_status;
	TestLogger *_log;

	bool _verbose = false;

	hrt_abstime _sent_time = 0;

	cert_test_status_s _report = {0};

	uORB::Publication<cert_test_status_s> _status_pub{ORB_ID(cert_test_status)};

	OrbDeviceReport<sensor_mag_s, MAG_INSTANCES> _mag_report =
		OrbDeviceReport<sensor_mag_s, MAG_INSTANCES>("sensor_mag",
							ORB_ID::sensor_mag,
							LISTENER_TIMEOUT,
							_log,
							_verbose);

	OrbDeviceReport<sensor_baro_s, BARO_INSTANCES> _baro_report =
		OrbDeviceReport<sensor_baro_s, BARO_INSTANCES>("sensor_baro",
							ORB_ID::sensor_baro,
							LISTENER_TIMEOUT,
							_log,
							_verbose);

	OrbDeviceReport<sensor_accel_s, IMU_INSTANCES> _accel_report =
		OrbDeviceReport<sensor_accel_s, IMU_INSTANCES>("sensor_accel",
							ORB_ID::sensor_accel,
							LISTENER_TIMEOUT,
							_log,
							_verbose);

	OrbDeviceReport<sensor_gyro_s, IMU_INSTANCES> _gyro_report =
		OrbDeviceReport<sensor_gyro_s, IMU_INSTANCES>("sensor_gyro",
							ORB_ID::sensor_gyro,
							LISTENER_TIMEOUT,
							_log,
							_verbose);

	OrbReport<airspeed_s, AIRSPEED_INSTANCES> _airspeed_report =
		OrbReport<airspeed_s, AIRSPEED_INSTANCES>("airspeed",
							ORB_ID::airspeed,
							LISTENER_TIMEOUT,
							_log,
							_verbose);

	OrbAdcReport<ADC_INSTANCES> _adc_report =
		OrbAdcReport<ADC_INSTANCES>("adc_report",
							ORB_ID::adc_report,
							LISTENER_TIMEOUT,
							_log,
							_verbose);

	OrbTelemReport<TELEM_INSTANCES> _telem_test =
		OrbTelemReport<TELEM_INSTANCES>("telem_test_status",
								ORB_ID::telem_test_status,
								LISTENER_TIMEOUT,
								_log,
								_verbose);
};



