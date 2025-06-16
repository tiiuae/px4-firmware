/****************************************************************************
 *
 *   Copyright (c) 2025 Technology Innovation Institute. All rights reserved.
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

/**
 * @file usr_shutdown.cpp
 *
 * User space interface for shutdown.
 */

#include <sys/boardctl.h>
#include <px4_platform_common/shutdown.h>

#include "shutdown_ioctl.h"

shutdown_handle_t px4_register_shutdown_hook()
{
	shutdowniocregister_t data = {PX4_ERROR};
	boardctl(SHUTDOWNIOCREGISTER, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int px4_unregister_shutdown_hook(shutdown_handle_t handle)
{
	shutdowniocunregister_t data = {handle, PX4_ERROR};
	boardctl(SHUTDOWNIOCUNREGISTER, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int px4_reboot_request(reboot_request_t request, uint32_t delay_us)
{
	shutdowniocreboot_t data = {request, delay_us, PX4_ERROR};
	boardctl(SHUTDOWNIOCREBOOT, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int px4_shutdown_request(uint32_t delay_us)
{
	shutdowniocshutdown_t data = {delay_us, PX4_ERROR};
	boardctl(SHUTDOWNIOCSHUTDOWN, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}
