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
#include <uORB/topics/px4io_status.h>

#include "orb_report.hpp"
#include "bg_proc.hpp"
#include "test_logger.hpp"
#include "can_test.hpp"
#include "test_config.hpp"

using namespace time_literals;

class CertTestStatus
{
public:
	CertTestStatus(uint32_t hw, BgProcExec *actuator, CanTest *can_test, TestLogger *log, bool verbose);

	~CertTestStatus() = default;

	void update();

private:
	static constexpr hrt_abstime REPORT_INTERVAL = 10_s;

	static constexpr hrt_abstime LISTENER_TIMEOUT = 3_s;

	static constexpr int SINGLE_INSTANCE = 1;
	static constexpr int BARO_INSTANCES = 2;
	static constexpr int IMU_INSTANCES = 3;
	static constexpr int ADC_FMU2_INSTANCES = 2;

	static constexpr int LATEST 		= 0;
	static constexpr int ERR_RECORD 	= 1;

	uint32_t _saluki_hw;
	BgProcExec *_actuator;
	CanTest *_can_test;
	TestLogger *_log;

	bool _verbose = false;

	hrt_abstime _sent_time = 0;

	cert_test_status_s _report = {0};

	uORB::Publication<cert_test_status_s> _status_pub{ORB_ID(cert_test_status)};

	OrbDeviceReport<sensor_mag_s, SINGLE_INSTANCE> _mag_report =
		OrbDeviceReport<sensor_mag_s, SINGLE_INSTANCE>("sensor_mag",
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

	OrbReport<airspeed_s, SINGLE_INSTANCE> _airspeed_report =
		OrbReport<airspeed_s, SINGLE_INSTANCE>("airspeed",
							ORB_ID::airspeed,
							LISTENER_TIMEOUT,
							_log,
							_verbose);

	const OrbAdcReport<SINGLE_INSTANCE>::ch_limits adc_limits = {
		{
			{
				9455625,
				{500, INT32_MAX},
				{INT32_MIN, 100},
				{2000, 4000},
				{2000, 4000},
			},
		}
	};

	const OrbAdcReport<ADC_FMU2_INSTANCES>::ch_limits adc_limits_fmu2 = {
		{
			{
				9455625,
				{1400, 3400},
				{INT32_MIN, 100},
				{1300, 3300},
				{12000, 14000},
			},
			{
				9455881,
				{8400, 9200},
				{8400, 9200},
				{8400, 9200},
				{8400, 9200},
			},
		}
	};

	OrbAdcReport<SINGLE_INSTANCE> _adc_report =
		OrbAdcReport<SINGLE_INSTANCE>("adc_report",
							ORB_ID::adc_report,
							LISTENER_TIMEOUT,
							adc_limits,
							_log,
							_verbose);

	OrbAdcReport<ADC_FMU2_INSTANCES> _adc_report_fmu2 =
		OrbAdcReport<ADC_FMU2_INSTANCES>("adc_report",
							ORB_ID::adc_report,
							LISTENER_TIMEOUT,
							adc_limits_fmu2,
							_log,
							_verbose);

	OrbTelemReport<SINGLE_INSTANCE> _telem_test =
		OrbTelemReport<SINGLE_INSTANCE>("telem_test_status",
								ORB_ID::telem_test_status,
								LISTENER_TIMEOUT,
								_log,
								_verbose);

	OrbReport<px4io_status_s, SINGLE_INSTANCE> _px4io_report =
		OrbReport<px4io_status_s, SINGLE_INSTANCE>("px4io_status",
							ORB_ID::px4io_status,
							LISTENER_TIMEOUT,
							_log,
							_verbose);

	struct orb_report_wrapper {
		uint32_t saluki_hw;
		OrbBase *orb;
		uint32_t *status;
	};

	const struct orb_report_wrapper _orb_report[20] = {
		{SALUKI_HW_ANY,		&_mag_report, _report.sensor_mag},
		{SALUKI_HW_ANY,		&_baro_report, _report.sensor_baro},
		{SALUKI_HW_ANY,		&_accel_report, _report.sensor_accel},
		{SALUKI_HW_ANY,		&_gyro_report, _report.sensor_gyro},
		{SALUKI_HW_ANY,		&_airspeed_report, _report.sensor_airspeed},
		{SALUKI_HW_V2,		&_adc_report, _report.adc_report},
		{SALUKI_HW_FMU2,	&_adc_report_fmu2, _report.adc_report},
		{SALUKI_HW_ANY,		&_telem_test, _report.telem_test},
		{SALUKI_HW_FMU2,	&_px4io_report, _report.px4io_status},
		{0, nullptr, nullptr},
	};

};



