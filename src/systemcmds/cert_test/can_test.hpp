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
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>

#include "orb_report.hpp"
#include "bg_proc.hpp"
#include "test_logger.hpp"

class CanTest
{
public:
	CanTest(TestLogger *log) : _log(log) {}

	~CanTest()
	{
		_stop_test = true;

		if (_thread != -1) {
			_log->log(TestLogger::INFO, "CAN test: wait thread to exit");
			pthread_join(_thread, NULL);
		}
	}

	int start();

	uint32_t status = OrbBase::STATUS_INIT;

private:
	static void* thread_func(void *args)
	{
		CanTest *obj = (CanTest *)args;
		obj->run_test();
		return NULL;
	}

	void run_test();

	TestLogger *_log;

	pthread_t _thread = -1;

	bool _stop_test = false;

	const char *CAN_SEQ[26] = {
		"can0", "123#10101010",
		"can0", "123#5A5A5A5A",
		"can1", "123#10101010",
		"can1", "123#5A5A5A5A",
		nullptr, nullptr,
		};
};
