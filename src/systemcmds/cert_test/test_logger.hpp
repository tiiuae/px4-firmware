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
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>

class TestLogger
{
public:
	TestLogger()
	{
		_disable_file = true;
	}

	TestLogger(const char* prefix, const char* dirname)
	{
		strncpy(_log_prefix, prefix, sizeof(_log_prefix) - 1);
		strncpy(_dirname, dirname, sizeof(_dirname) - 1);

		_disable_file = false;
	}

	~TestLogger()
	{
		if (_mutex != nullptr) pthread_mutex_destroy(_mutex);
		close(_fd);
	}

	int init()
	{
		_mutex = new pthread_mutex_t;
		if (pthread_mutex_init(_mutex, NULL)) {
			PX4_ERR("mutex init failed");
			return 1;
		}

		if (_disable_file) {
			return 0;
		}

		char dirpath[STR_MAX_LEN] = {0};

		int i;
		// find first unused path name
		for (i = 0; i < 999; i++) {
			snprintf(dirpath, sizeof(dirpath), "/fs/microsd/log/%s%03d", _dirname, i);

			struct stat sb = {0};
			if (stat(dirpath, &sb) == 0) {
				continue;
			}

			if (errno != ENOENT) {
				PX4_ERR("'%s' stat failed: %d", dirpath, errno);
				return 1;
			}

			// unused path name found
			break;
		}

		if (mkdir(dirpath, PX4_O_MODE_666) != 0) {
			PX4_ERR("'%s' creation failed: %d", dirpath, errno);
			return 1;
		}

		snprintf(_logfile, sizeof(_logfile), "/fs/microsd/log/%s%03d/log100.ulg", _dirname, i);

		_fd = open(_logfile, O_WRONLY | O_CREAT | O_TRUNC | O_SYNC, PX4_O_MODE_666);

		if (_fd < 0) {
			PX4_ERR("'%s' create failed: %d", _logfile, errno);
			return 1;
		}

		PX4_INFO("logfile created: %s", _logfile);

		return 0;
	}

	void log(bool error, const char *fmt, ...)
	{
		if (_mutex == nullptr) {
			PX4_ERR("log not initialized");
			return;
		}

		pthread_mutex_lock(_mutex);

		va_list	ap;
		va_start(ap, fmt);
		vsnprintf(_logbuf, STR_MAX_LEN, fmt, ap);
		va_end(ap);

		if (error) {
			PX4_ERR("%s", _logbuf);
		} else {
			PX4_INFO("%s", _logbuf);
		}

		if (_disable_file) {
			pthread_mutex_unlock(_mutex);
			return;
		}

		if (_fd < 0) {
			PX4_ERR("log not initialized");
			pthread_mutex_unlock(_mutex);
			return;
		}

		const char *type_str = "INFO";
		if (error) {
			type_str = "ERROR";
		}

		if (dprintf(_fd, "%s %s ", type_str, _log_prefix) < 0) {
			PX4_ERR("'%s' write failed: %d", _logfile, errno);
			pthread_mutex_unlock(_mutex);
			return;
		}

		if (write(_fd, _logbuf, strlen(_logbuf)) < 0) {
			PX4_ERR("'%s' write failed: %d", _logfile, errno);
			pthread_mutex_unlock(_mutex);
			return;
		}

		if (dprintf(_fd, "\n") < 0) {
			PX4_ERR("'%s' write failed: %d", _logfile, errno);
			pthread_mutex_unlock(_mutex);
			return;
		}

		fsync(_fd);

		pthread_mutex_unlock(_mutex);
	}

	static constexpr bool INFO = false;
	static constexpr bool ERR = true;

private:
	static constexpr int STR_MAX_LEN = 128;
	static constexpr int LOG_MAX_LEN = 1024;

	bool _disable_file = false;

	char _log_prefix[STR_MAX_LEN] = {0};
	char _dirname[STR_MAX_LEN] = {0};
	char _logfile[STR_MAX_LEN] = {0};

	char _logbuf[LOG_MAX_LEN] = {0};

	int _fd = -1;

	pthread_mutex_t *_mutex = nullptr;

};


