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
 * @file shutdown_ioctl.h
 *
 * User space - kernel space interface for shutdown
 */

#pragma once

#include <arch/inttypes.h>
#include <px4_platform/board_ctrl.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/shutdown.h>

#define _SHUTDOWNIOC(_n) (_PX4_IOC(_SHUTDOWNIOCBASE, _n))

#define SHUTDOWNIOCREGISTER _SHUTDOWNIOC(1)
typedef struct shutdowniocregister {
	int ret;
} shutdowniocregister_t;


#define SHUTDOWNIOCUNREGISTER _SHUTDOWNIOC(2)
typedef struct shutdowniocunregister {
	shutdown_handle_t handle;
	int ret;
} shutdowniocunregister_t;

#define SHUTDOWNIOCREBOOT _SHUTDOWNIOC(3)
typedef struct shutdowniocreboot {
	reboot_request_t request;
	uint32_t delay_us;
	int ret;
} shutdowniocreboot_t;


#define SHUTDOWNIOCSHUTDOWN _SHUTDOWNIOC(4)
typedef struct shutdowniocshutdown {
	uint32_t delay_us;
	int ret;
} shutdowniocshutdown_t;

#define SHUTDOWNIOCSETFORCE _SHUTDOWNIOC(5)
typedef struct shutdowniocsetforce {
	bool force;
	int ret;
} shutdowniocsetforce_t;
