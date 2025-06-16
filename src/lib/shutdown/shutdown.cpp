/****************************************************************************
 *
 * Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file shutdown.cpp
 * Implementation of the API declared in px4_shutdown.h.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>

#include <drivers/drv_hrt.h>

#ifndef MODULE_NAME
#define MODULE_NAME "shutdown"
#endif

#include <px4_platform_common/external_reset_lockout.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/shutdown.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/shutdown_event.h>


#include <stdint.h>
#include <errno.h>

#ifdef __PX4_NUTTX
#include <nuttx/board.h>
#include <sys/boardctl.h>
#endif

using namespace time_literals;

#if defined(CONFIG_SCHED_WORKQUEUE) || (!defined(CONFIG_BUILD_FLAT) && defined(CONFIG_LIBC_USRWORK))

static struct work_s shutdown_work = {};

#define SHUTDOWN_ARG_IN_PROGRESS (1<<0)
#define SHUTDOWN_ARG_REBOOT (1<<1)
#define SHUTDOWN_ARG_TO_BOOTLOADER (1<<2)
#define SHUTDOWN_ARG_TO_ISP (1<<3)
#define SHUTDOWN_ARG_BL_CONTINUE_BOOT (1<<4)

static uint8_t shutdown_args = 0;

static px4_sem_t shutdown_hook_lock;

static constexpr int max_shutdown_hooks = 10;
static bool shutdown_hooks[max_shutdown_hooks] = {false};

static hrt_abstime shutdown_time_us = 0;
static constexpr hrt_abstime shutdown_timeout_us =
	5_s; ///< force shutdown after this time if modules do not respond in time

static uORB::Publication<shutdown_event_s> *shutdown_event{nullptr};

void shutdown_init()
{
	px4_mutex_init(&shutdown_hook_lock, 0);

	px4_sem_wait(&shutdown_hook_lock);

	if (!shutdown_event) {
		shutdown_event = new uORB::Publication<shutdown_event_s>(ORB_ID(shutdown_event));

		if (shutdown_event) {
			if (!shutdown_event->advertise()) {
				PX4_ERR("Failed to advertise shutdown event topic\n");
			}

		} else {
			PX4_ERR("Failed to create shutdown event topic\n");
		}
	}

	px4_sem_post(&shutdown_hook_lock);
}

shutdown_handle_t px4_register_shutdown_hook()
{
	px4_sem_wait(&shutdown_hook_lock);

	if (shutdown_event) {
		for (int i = 0; i < max_shutdown_hooks; ++i) {
			if (!shutdown_hooks[i]) {
				shutdown_hooks[i] = true;
				PX4_DEBUG("Registered shutdown hook with handle %i\n", i);
				px4_sem_post(&shutdown_hook_lock);
				return i;
			}
		}

		PX4_ERR("Failed to register shutdown hook\n");
	}

	px4_sem_post(&shutdown_hook_lock);

	return -ENOMEM;
}

int px4_unregister_shutdown_hook(shutdown_handle_t handle)
{
	px4_sem_wait(&shutdown_hook_lock);

	if (handle >= 0 && handle < max_shutdown_hooks && shutdown_hooks[handle]) {
		shutdown_hooks[handle] = false;
		PX4_DEBUG("Unregistered shutdown hook for handle %i\n", handle);
		px4_sem_post(&shutdown_hook_lock);
		return 0;
	}

	px4_sem_post(&shutdown_hook_lock);

	PX4_DEBUG("Tried to unregister shutdown hook from unknown handle: %i\n", handle);

	return -EINVAL;
}

/**
 * work queue callback method to shutdown.
 * @param arg unused
 */
static void shutdown_worker(void *arg)
{
	PX4_DEBUG("shutdown worker");
	bool done = true;

	// wait responses from processes registered in shutdown_hook
	px4_sem_wait(&shutdown_hook_lock);

	for (int i = 0; i < max_shutdown_hooks; ++i) {
		if (shutdown_hooks[i]) {
			PX4_DEBUG("Uncomplete shutdown hook for handle %i\n", i);
			done = false;
			break;
		}
	}

	const hrt_abstime now = hrt_absolute_time();
	const bool delay_elapsed = (now > shutdown_time_us);

	if (delay_elapsed && (done || (now > (shutdown_time_us + shutdown_timeout_us)))) {
		if (shutdown_args & SHUTDOWN_ARG_REBOOT) {
#if defined(CONFIG_BOARDCTL_RESET)
			PX4_INFO_RAW("Reboot NOW.");

			if (shutdown_event) {
				shutdown_event->unadvertise();
				delete shutdown_event;
				shutdown_event = nullptr;
			}

			if (shutdown_args & SHUTDOWN_ARG_TO_BOOTLOADER) {
				if (shutdown_args & SHUTDOWN_ARG_BL_CONTINUE_BOOT) {
					boardctl(BOARDIOC_RESET, (uintptr_t)REBOOT_TO_BOOTLOADER_CONTINUE);

				} else {
					boardctl(BOARDIOC_RESET, (uintptr_t)REBOOT_TO_BOOTLOADER);

				}

			} else if (shutdown_args & SHUTDOWN_ARG_TO_ISP) {
				boardctl(BOARDIOC_RESET, (uintptr_t)REBOOT_TO_ISP);

			} else {
				boardctl(BOARDIOC_RESET, (uintptr_t)REBOOT_REQUEST);
			}

#else
			PX4_PANIC("board reset not available");
#endif

			px4_sem_post(&shutdown_hook_lock); // must NEVER come here

		} else {
#if defined(BOARD_HAS_POWER_CONTROL)
			PX4_INFO_RAW("Powering off NOW.");
#if defined(CONFIG_BOARDCTL_POWEROFF)
			boardctl(BOARDIOC_POWEROFF, 0);
#else
			board_power_off(0);
#endif
#elif defined(__PX4_POSIX)
			// simply exit on posix if real shutdown (poweroff) not available
			PX4_INFO_RAW("Exiting NOW.");
			system_exit(0);
#else
			PX4_PANIC("board shutdown not available");
#endif
		}

	} else {
		px4_sem_post(&shutdown_hook_lock);
		PX4_DEBUG("Rescheduling shutdown worker\n");
		work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, USEC2TICK(10000));
	}
}

#if defined(CONFIG_BOARDCTL_RESET)
int px4_reboot_request(reboot_request_t request, uint32_t delay_us)
{
	if (shutdown_args & SHUTDOWN_ARG_IN_PROGRESS || shutdown_args & SHUTDOWN_ARG_REBOOT) {
		return 0;
	}

	shutdown_event_s event_msg{};
	event_msg.timestamp = hrt_absolute_time();

	if (shutdown_event && !shutdown_event->publish(event_msg)) {
		PX4_ERR("Failed to publish shutdown uorb %d (%s)\n", errno, strerror(errno));
	}

	shutdown_args |= SHUTDOWN_ARG_REBOOT;

	if (request == REBOOT_TO_BOOTLOADER) {
		shutdown_args |= SHUTDOWN_ARG_TO_BOOTLOADER;

	} else if (request == REBOOT_TO_BOOTLOADER_CONTINUE) {
		shutdown_args |= (SHUTDOWN_ARG_TO_BOOTLOADER | SHUTDOWN_ARG_BL_CONTINUE_BOOT);

	} else if (request == REBOOT_TO_ISP) {
		shutdown_args |= SHUTDOWN_ARG_TO_ISP;
	}

	shutdown_time_us = hrt_absolute_time();

	if (delay_us > 0) {
		shutdown_time_us += delay_us;
	}

	work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, 1);

	return 0;
}
#endif // CONFIG_BOARDCTL_RESET

#if defined(BOARD_HAS_POWER_CONTROL) || defined(__PX4_POSIX)
int px4_shutdown_request(uint32_t delay_us)
{
	if (shutdown_args & SHUTDOWN_ARG_IN_PROGRESS) {
		return 0;
	}

	shutdown_event_s event_msg{};
	event_msg.timestamp = hrt_absolute_time();

	if (shutdown_event && !shutdown_event->publish(event_msg)) {
		PX4_ERR("Failed to publish shutdown uorb %d (%s)\n", errno, strerror(errno));
	}

	shutdown_args |= SHUTDOWN_ARG_IN_PROGRESS;

	shutdown_time_us = hrt_absolute_time();

	if (delay_us > 0) {
		shutdown_time_us += delay_us;
	}

	work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, 1);
	return 0;
}
#endif // BOARD_HAS_POWER_CONTROL

#endif // CONFIG_SCHED_WORKQUEUE)
