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

#include <px4_platform_common/module.h>
#include "test_logger.hpp"

class BgProcExec
{
public:
	BgProcExec(const char *name,
				const char *cmd,
				const char **cmd_argv,
				bool send_sigterm,
				TestLogger *log) :
		_name(name),
		_cmd(cmd),
		_cmd_argv(cmd_argv),
		_send_sigterm(send_sigterm),
		_log(log)
	{
		_bg_task = px4_exec(_cmd, (char *const *)_cmd_argv, nullptr, 0);

		_log->log(_log->INFO, "%s started (%d)", _name, _bg_task);
	}

	~BgProcExec()
	{
		if (!_send_sigterm) {
			return;
		}

		if (kill(_bg_task, SIGTERM)) {
			_log->log(_log->ERR, "sending SIGTERM to %s failed: %d", _name, errno);
			return;
		}
		_log->log(_log->INFO, "%s (%d) killed", _name, _bg_task);
	}

	bool rerun(const char **cmd_argv)
	{
		while (get_pid(false) != -1) {
			pthread_yield();
		}

		if (cmd_argv) {
			_cmd_argv = cmd_argv;
		}

		_bg_task = px4_exec(_cmd, (char *const *)_cmd_argv, nullptr, 0);

		if (_bg_task < 0) {
			return false;
		}

		return true;
	}

	px4_task_t get_pid(bool verbose)
	{
	  	struct tcb_s *dtcb = nullptr;

		dtcb = nxsched_get_tcb(_bg_task);

		if (!dtcb)
		{
			if (verbose) _log->log(_log->ERR, "%s process not found", _name);
			return -1;
		}

		return _bg_task;
	}

	static constexpr bool KILL = true;
	static constexpr bool NO_KILL = false;

private:
	px4_task_t _bg_task = -1;

	const char *_name;
	const char *_cmd;
	const char **_cmd_argv;
	bool _send_sigterm;
	TestLogger *_log;
};
