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
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/telem_test_status.h>
#include "test_logger.hpp"

class OrbBase
{
public:
	static constexpr int SINGLE_INSTANCE			= 1;

	static constexpr uint32_t STATUS_INIT			= 0x0;
	static constexpr uint32_t STATUS_OK				= 0x1;
	static constexpr uint32_t STATUS_NOT_UPDATED	= 0x2;
	static constexpr uint32_t STATUS_INV_INSTANCE	= 0x4;
	static constexpr uint32_t STATUS_INV_VALUE		= 0x8;
	static constexpr uint32_t STATUS_TEST_FAIL		= 0x10;
	static constexpr uint32_t STATUS_NOT_RUNNING	= 0x20;
	static constexpr uint32_t STATUS_INV_CONFIG		= 0x40;

	virtual void get_latest() = 0;
	virtual uint32_t get_result() = 0;
};

template<class T, int SIZE>
class OrbReport : public OrbBase
{
public:
	OrbReport(const char *name, ORB_ID id, uint64_t timeout, TestLogger *log, bool verbose) :
		_name(name),
		_sub(id),
		_timeout(timeout),
		_logger(log),
		_verbose(verbose)
	{
		_report = new T[SIZE];
		memset(_report, 0, sizeof(T) * SIZE);

		if (_report == nullptr) {
			_logger->log(TestLogger::ERR, "%s: failed to allocate memory for report", _name);
		}
	}

	virtual ~OrbReport()
	{
		delete[] _report;
	}

	void get_latest()
	{
		if (_report == nullptr) {
			_logger->log(TestLogger::ERR, "%s: report not allocated", _name);
			return;
		}

		for (int i = 0; i < _sub.size(); i++) {
			if (!_sub[i].updated()) {
				continue;
			}

			_sub[i].copy(&_report[i]);
		}
	}

	virtual uint32_t get_result()
	{
		uint32_t old_res = _result;

		_result = STATUS_INIT;

		if (_report == nullptr) {
			_logger->log(TestLogger::ERR, "%s: report not allocated", _name);
			return _result;
		}

		if (_sub.advertised_count() != _sub.size()) {
			if (!(old_res & STATUS_INV_INSTANCE)) {
				_logger->log(TestLogger::ERR, "%s: invalid instance count %d/%d",
					_name, _sub.size(), _sub.advertised_count());
			}
			_result |= STATUS_INV_INSTANCE;
			return _result;
		}

		for (int i = 0; i < _sub.size(); i++) {
			if (hrt_absolute_time() - _report[i].timestamp > _timeout) {
				if (!(old_res & STATUS_NOT_UPDATED)) {
					_logger->log(TestLogger::ERR, "%s [%d]: timeout ts %ld", _name, i, _report[i].timestamp);
				}
				_result |= STATUS_NOT_UPDATED;
				continue;
			}
		}

		if (!(_result & STATUS_NOT_UPDATED)) {
			_result |= STATUS_OK;
		}

		return _result;
	}

	T* get_report(unsigned instance = 0) { return &_report[instance]; }

protected:
	const char *_name;

	uORB::SubscriptionMultiArray<T, SIZE> _sub;

	T *_report = nullptr;

	uint64_t _timeout;

	TestLogger *_logger;

	bool _verbose = false;

	uint32_t _result = 0;
};

template<class T, int SIZE>
class OrbDeviceReport : public OrbReport<T, SIZE>
{
public:
	OrbDeviceReport(const char *name, ORB_ID id, uint64_t timeout, TestLogger *log, bool verbose) :
		OrbReport<T, SIZE>(name, id, timeout, log, verbose)
	{}

	~OrbDeviceReport() = default;

	virtual uint32_t get_result()
	{
		uint32_t old_res = this->_result;

		OrbReport<T, SIZE>::get_result();

		// get_result() above does not print device_id
		if(this->_result & OrbBase::STATUS_NOT_UPDATED && !(old_res & OrbBase::STATUS_NOT_UPDATED)) {
			for (int i = 0; i < this->_sub.size(); i++) {
				if (hrt_absolute_time() - this->_report[i].timestamp > this->_timeout) {
					this->_logger->log(TestLogger::ERR, "%s [i]: timeout dev 0x%x", this->_name, i, this->_report[i].device_id);
				}
			}
		}
		return this->_result;
	}
};

template<int SIZE>
class OrbAdcReport : public OrbDeviceReport<adc_report_s, SIZE>
{
public:
	struct ch_limits {
		struct adc_limit {
			uint32_t device_id;
			int32_t ch_0[2];
			int32_t ch_1[2];
			int32_t ch_2[2];
			int32_t ch_3[2];
		};

		struct adc_limit adc[SIZE];
	};

	OrbAdcReport(const char *name, ORB_ID id, uint64_t timeout, const struct ch_limits &limits, TestLogger *log, bool verbose) :
		OrbDeviceReport<adc_report_s, SIZE>(name, id, timeout, log, verbose),
		_limits(limits)
	{
		if (this->_verbose) {
			for (int i = 0; i < SIZE; i++) {
				this->_logger->log(TestLogger::INFO, "device %d ADC limits [%d]: %d/%d, %d/%d, %d/%d, %d/%d",
					limits.adc[i].device_id, i,
					limits.adc[i].ch_0[0], limits.adc[i].ch_0[1],
					limits.adc[i].ch_1[0], limits.adc[i].ch_1[1],
					limits.adc[i].ch_2[0], limits.adc[i].ch_2[1],
					limits.adc[i].ch_3[0], limits.adc[i].ch_3[1]);
			}
		}
	}

	~OrbAdcReport() = default;

	virtual uint32_t get_result()
	{
		uint32_t old_res = this->_result;

		OrbDeviceReport<adc_report_s, SIZE>::get_result();

		for (int i = 0; i < this->_sub.size(); i++) {
			int32_t *raw_data = this->_report[i].raw_data;
			const struct ch_limits::adc_limit *value_limit = &_limits.adc[i];

			if (value_limit->device_id != this->_report[i].device_id) {
				if (!(old_res & OrbBase::STATUS_INV_CONFIG)) {
					this->_logger->log(TestLogger::ERR, "%s: invalid device_id %d",
						this->_name, this->_report[i].device_id);
				}

				this->_result |= OrbBase::STATUS_INV_CONFIG;
				this->_result &= ~(OrbBase::STATUS_OK);
				return this->_result;
			}

			if (this->_verbose) {
				this->_logger->log(TestLogger::INFO, "%s: %d [%d, %d, %d, %d]",
					this->_name, this->_report[i].device_id,
					raw_data[0], raw_data[1], raw_data[2], raw_data[3]);
			}

			if (raw_data[0] < value_limit->ch_0[0] || raw_data[0] > value_limit->ch_0[1] ||
				raw_data[1] < value_limit->ch_1[0] || raw_data[1] > value_limit->ch_1[1] ||
				raw_data[2] < value_limit->ch_2[0] || raw_data[2] > value_limit->ch_2[1] ||
				raw_data[3] < value_limit->ch_3[0] || raw_data[3] > value_limit->ch_3[1]) {

				if (!(old_res & OrbBase::STATUS_INV_VALUE)) {
					this->_logger->log(TestLogger::ERR, "%s: invalid value [%d, %d, %d, %d]",
						this->_name, raw_data[0], raw_data[1], raw_data[2], raw_data[3]);
				}

				this->_result |= OrbBase::STATUS_INV_VALUE;
				this->_result &= ~(OrbBase::STATUS_OK);
			}
		}

		return this->_result;
	}
private:
	const struct ch_limits &_limits;
};

class OrbTelemReport : public OrbReport<telem_test_status_s, OrbBase::SINGLE_INSTANCE>
{
public:
	OrbTelemReport(const char *name, ORB_ID id, uint64_t timeout, TestLogger *log, bool verbose) :
		OrbReport<telem_test_status_s, OrbBase::SINGLE_INSTANCE>(name, id, timeout, log, verbose)
	{}

	~OrbTelemReport() = default;

	virtual uint32_t get_result()
	{
		uint32_t old_res = this->_result;
		uint32_t *telem_status = this->_report->status;

		OrbReport<telem_test_status_s, OrbBase::SINGLE_INSTANCE>::get_result();

		for (int i = 0; i < TELEM_UART_COUNT; i++) {
			if (!(telem_status[i] & TELEM_STATUS_OK) ||
				telem_status[i] & TELEM_STATUS_WRITE_ERR ||
				telem_status[i] & TELEM_STATUS_READ_ERR) {

					if (!(old_res & OrbBase::STATUS_TEST_FAIL)) {
						this->_logger->log(TestLogger::ERR, "%s: test failure [0x%x, 0x%x, 0x%x]",
							this->_name, telem_status[0], telem_status[1], telem_status[2]);
					}

					this->_result |= OrbBase::STATUS_TEST_FAIL;
					this->_result &= ~(OrbBase::STATUS_OK);
				}
		}

		return this->_result;
	}

	virtual bool error_record()
	{
		for (int i = 0; i < TELEM_UART_COUNT; i++) {
			if (this->_report->status[i] & TELEM_ERROR_TYPE_MASK) {
				return true;
			}
		}

		return false;
	}

	static constexpr int TELEM_UART_COUNT = 3;

	static constexpr int TELEM_STATUS_IDLE 			= 0x1;
	static constexpr int TELEM_STATUS_OK 			= 0x2;

	// Active error indicators. Rest of bits are error record for active and
	// recovered errors.
	static constexpr int TELEM_STATUS_WRITE_ERR 	= 0x4;
	static constexpr int TELEM_STATUS_READ_ERR 		= 0x8;

	static constexpr int TELEM_ERROR_TYPE_MASK		= ~(TELEM_STATUS_IDLE |
														TELEM_STATUS_OK |
														TELEM_STATUS_WRITE_ERR |
														TELEM_STATUS_READ_ERR);
};

