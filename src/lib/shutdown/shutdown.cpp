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

//#include <px4_platform_common/external_reset_lockout.h>
#include <px4_platform_common/log.h>

#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/shutdown_event.h>
#include <uORB/topics/shutdown_ack.h>

#include <stdint.h>
#include <errno.h>
#include <pthread.h>

#ifdef __PX4_NUTTX
#include <nuttx/board.h>
#include <sys/boardctl.h>
#endif

using namespace time_literals;

static pthread_mutex_t shutdown_mutex =
	PTHREAD_MUTEX_INITIALIZER; // protects access to shutdown_hooks & shutdown_lock_counter
static uint8_t shutdown_lock_counter = 0;

int px4_shutdown_lock()
{
	int ret = pthread_mutex_lock(&shutdown_mutex);

	if (ret == 0) {
		++shutdown_lock_counter;
//		px4_indicate_external_reset_lockout(LockoutComponent::SystemShutdownLock, true);
		return pthread_mutex_unlock(&shutdown_mutex);
	}

	return ret;
}

int px4_shutdown_unlock()
{
	int ret = pthread_mutex_lock(&shutdown_mutex);

	if (ret == 0) {
		if (shutdown_lock_counter > 0) {
			--shutdown_lock_counter;

			if (shutdown_lock_counter == 0) {
//				px4_indicate_external_reset_lockout(LockoutComponent::SystemShutdownLock, false);
			}

		} else {
			PX4_ERR("unmatched number of px4_shutdown_unlock() calls");
		}

		return pthread_mutex_unlock(&shutdown_mutex);
	}

	return ret;
}

#if defined(CONFIG_SCHED_WORKQUEUE) || (!defined(CONFIG_BUILD_FLAT) && defined(CONFIG_LIBC_USRWORK))

static struct work_s shutdown_work = {};
static uint16_t shutdown_counter = 0; ///< count how many times the shutdown worker was executed

#define SHUTDOWN_ARG_IN_PROGRESS (1<<0)
#define SHUTDOWN_ARG_REBOOT (1<<1)
#define SHUTDOWN_ARG_TO_BOOTLOADER (1<<2)
#define SHUTDOWN_ARG_TO_ISP (1<<3)
#define SHUTDOWN_ARG_BL_CONTINUE_BOOT (1<<4)

static uint8_t shutdown_args = 0;

static constexpr int max_shutdown_hooks = 10;
static shutdown_hook_t shutdown_hooks[max_shutdown_hooks] = {0};

static hrt_abstime shutdown_time_us = 0;
static constexpr hrt_abstime shutdown_timeout_us =
	5_s; ///< force shutdown after this time if modules do not respond in time

class ShutdownAckCallback : public uORB::SubscriptionCallback
{
public:
	ShutdownAckCallback() :
		uORB::SubscriptionCallback(ORB_ID(shutdown_ack))
	{}

	void call()
	{
		if (updated()) {
			shutdown_ack_s ack_msg;
			copy(&ack_msg);

			if (ack_msg.pid > 0) {
				pthread_mutex_lock(&shutdown_mutex);

				for (int i = 0; i < max_shutdown_hooks; ++i) {
					if (shutdown_hooks[i] == ack_msg.pid) {
						PX4_DEBUG("Shutdown ack from pid %i\n", ack_msg.pid);
						shutdown_hooks[i] = -1;
						pthread_mutex_unlock(&shutdown_mutex);
						return;
					}
				}

				pthread_mutex_unlock(&shutdown_mutex);

			} else {
				PX4_WARN("Shutdown ack from unknown pid: %i\n", ack_msg.pid);
			}
		}

		return;
	}
};

static ShutdownAckCallback *shutdown_ack_callback{nullptr};

static uORB::Publication<shutdown_event_s> *shutdown_event{nullptr};

int px4_register_shutdown_hook(shutdown_hook_t hook)
{
	// create and advertise shutdown event topic if not done yet
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

	// subscribe shutdown ack topic if not done yet
	if (!shutdown_ack_callback) {
		shutdown_ack_callback = new ShutdownAckCallback();

		if (shutdown_ack_callback) {
			if (!shutdown_ack_callback->registerCallback()) {
				PX4_ERR("Failed to register shutdown ack callback\n");
			}

		} else {
			PX4_ERR("Failed to create ShutdownAckCallback\n");
		}
	}

	pthread_mutex_lock(&shutdown_mutex);

	for (int i = 0; i < max_shutdown_hooks; ++i) {
		if (shutdown_hooks[i] <= 0) {
			shutdown_hooks[i] = hook;
			PX4_DEBUG("Registered shutdown hook for pid %i\n", hook);
			pthread_mutex_unlock(&shutdown_mutex);
			return 0;
		}
	}

	pthread_mutex_unlock(&shutdown_mutex);

	return -ENOMEM;
}

int px4_unregister_shutdown_hook(shutdown_hook_t hook)
{
	pthread_mutex_lock(&shutdown_mutex);

	for (int i = 0; i < max_shutdown_hooks; ++i) {
		if (shutdown_hooks[i] == hook) {
			shutdown_hooks[i] = -1;
			PX4_DEBUG("Unregistered shutdown hook for pid %i\n", hook);
			pthread_mutex_unlock(&shutdown_mutex);
			return 0;
		}
	}

	pthread_mutex_unlock(&shutdown_mutex);

	PX4_DEBUG("Tried to unregister shutdown hook from unknown pid: %i\n", hook);

	return -EINVAL;
}

/**
 * work queue callback method to shutdown.
 * @param arg unused
 */
static void shutdown_worker(void *arg)
{
	PX4_DEBUG("shutdown worker (%i)", shutdown_counter);
	bool done = true;

	// wait responses from PIDs registered in shutdown_hook
	pthread_mutex_lock(&shutdown_mutex);

	for (int i = 0; i < max_shutdown_hooks; ++i) {
		if (shutdown_hooks[i] > 0) {
			PX4_DEBUG("Uncomplete shutdown hook for pid %i\n", shutdown_hooks[i]);
			done = false;
		}
	}

	const hrt_abstime now = hrt_absolute_time();
	const bool delay_elapsed = (now > shutdown_time_us);

	if (delay_elapsed && ((done && shutdown_lock_counter == 0) || (now > (shutdown_time_us + shutdown_timeout_us)))) {
		if (shutdown_args & SHUTDOWN_ARG_REBOOT) {
#if defined(CONFIG_BOARDCTL_RESET)
			PX4_INFO_RAW("Reboot NOW.");

			// remove shutdown ack callback object
			if (shutdown_ack_callback) {
				shutdown_ack_callback->unregisterCallback();
				delete shutdown_ack_callback;
			}

			if (shutdown_event) {
				shutdown_event->unadvertise();
				delete shutdown_event;
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

		pthread_mutex_unlock(&shutdown_mutex); // must NEVER come here

	} else {
		PX4_DEBUG("Rescheduling shutdown worker\n");
		pthread_mutex_unlock(&shutdown_mutex);
		work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, USEC2TICK(10000));
	}
}

#if defined(CONFIG_BOARDCTL_RESET)
int px4_reboot_request(reboot_request_t request, uint32_t delay_us)
{
	// create and advertise shutdown event topic if not done yet
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

	pthread_mutex_lock(&shutdown_mutex);

	if (shutdown_args & SHUTDOWN_ARG_IN_PROGRESS) {
		pthread_mutex_unlock(&shutdown_mutex);
		return 0;
	}

	shutdown_event_s event_msg{};
	event_msg.timestamp = hrt_absolute_time();
	event_msg.triggered = true;

	if (!shutdown_event->publish(event_msg)) {
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
	pthread_mutex_unlock(&shutdown_mutex);
	return 0;
}
#endif // CONFIG_BOARDCTL_RESET

#if defined(BOARD_HAS_POWER_CONTROL) || defined(__PX4_POSIX)
int px4_shutdown_request(uint32_t delay_us)
{
	// create and advertise shutdown event topic if not done yet
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

	pthread_mutex_lock(&shutdown_mutex);

	if (shutdown_args & SHUTDOWN_ARG_IN_PROGRESS) {
		pthread_mutex_unlock(&shutdown_mutex);
		return 0;
	}

	shutdown_event_s event_msg{};
	event_msg.timestamp = hrt_absolute_time();
	event_msg.triggered = true;

	if (!shutdown_event->publish(event_msg)) {
		PX4_ERR("Failed to publish shutdown uorb %d (%s)\n", errno, strerror(errno));
	}

	shutdown_args |= SHUTDOWN_ARG_IN_PROGRESS;

	shutdown_time_us = hrt_absolute_time();

	if (delay_us > 0) {
		shutdown_time_us += delay_us;
	}

	work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, 1);
	pthread_mutex_unlock(&shutdown_mutex);
	return 0;
}

#endif // BOARD_HAS_POWER_CONTROL

#endif // CONFIG_SCHED_WORKQUEUE)
