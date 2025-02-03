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

#include "can_test.hpp"
#include "netutils/netlib.h"

int CanTest::start()
{
	pthread_attr_t th_attr;

	if (netlib_ifup("can0") == -1) {
		_log->log(TestLogger::ERR, "netlib_ifup can0");
		return 1;
	} else {
		_log->log(TestLogger::INFO, "Enable can0");
	}

	if (netlib_ifup("can1") == -1) {
		_log->log(TestLogger::ERR, "netlib_ifup can1");
		return 1;
	} else {
		_log->log(TestLogger::INFO, "Enable can1");
	}

	if (pthread_attr_init(&th_attr)) {
		_log->log(TestLogger::ERR, "pthread_attr_init");
		return 1;
	}

	if (pthread_attr_setstacksize(&th_attr, 4096)) {
		_log->log(TestLogger::ERR, "pthread_attr_setstacksize");
		return 1;
	}

	if (pthread_create(&_thread, &th_attr, CanTest::thread_func, this))
	{
		_log->log(TestLogger::ERR, "pthread_create: %d", errno);
		return 1;
	}

	return 0;
}

void CanTest::run_test()
{
	_log->log(TestLogger::INFO, "CAN test thread started");

	const char *cmd_argv[] = {0, "can0", "123#10101010",  nullptr};
	BgProcExec *can_test = new BgProcExec("cansend", "cansend", cmd_argv, BgProcExec::NO_KILL, _log);

	while (!_stop_test) {
		bool failure = false;

		for (int i = 0; CanTest::CAN_SEQ[i] != nullptr; i += 2) {
			cmd_argv[1] = CanTest::CAN_SEQ[i];
			cmd_argv[2] = CanTest::CAN_SEQ[i + 1];

			if (can_test->rerun(cmd_argv)) {
				px4_usleep(1000);
				continue;
			}

			if (!(status & OrbBase::STATUS_NOT_RUNNING)) {
				_log->log(TestLogger::ERR, "cansend process creation failed");
			}

			status = OrbBase::STATUS_NOT_RUNNING;
			failure = true;
		}

		if (!failure) {
			status = OrbBase::STATUS_OK;
		}

		px4_usleep(1000);
	}

	status = OrbBase::STATUS_INIT;

	delete can_test;

	return;
}

