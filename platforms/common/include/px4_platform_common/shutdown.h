/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file shutdown.h
 * Power-related API
 */

#pragma once

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>
#include <inttypes.h>

#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/shutdown_event.h>

__BEGIN_DECLS

/*
 * Handle of the process which registered to listen shutdown event. (@see px4_register_shutdown_hook())
*/
typedef int shutdown_handle_t;


/**
 * Register a method that should be called when powering off (and also on reboot).
 * @return handle on success, <0 on error
 */
__EXPORT shutdown_handle_t px4_register_shutdown_hook();


/**
 * Unregister a shutdown hook
 * @param handle to be removed
 * @return 0 on success, <0 on error
 */
__EXPORT int px4_unregister_shutdown_hook(shutdown_handle_t hook);

/** Types of reboot requests for PX4 */
typedef enum {
	REBOOT_REQUEST = 0,          ///< Normal reboot
	REBOOT_TO_BOOTLOADER = 1,    ///< Reboot to PX4 bootloader
	REBOOT_TO_ISP = 2,           ///< Reboot to ISP bootloader
	REBOOT_TO_BOOTLOADER_CONTINUE = 3, // < Reboot to bootloader and continue TODO!
} reboot_request_t;

/**
 * Request the system to reboot.
 * Note the following:
 * - The system might not support reboot. In that case -EINVAL will
 *   be returned.
 * - The system might not shutdown immediately, so expect this method to return even
 *   on success.
 * @param to_bootloader reboot into bootloader mode (only used if reboot is true)
 * @param delay_us optional delay in microseconds
 * @return 0 on success, <0 on error
 */
#if defined(CONFIG_BOARDCTL_RESET)
__EXPORT int px4_reboot_request(reboot_request_t request = REBOOT_REQUEST, uint32_t delay_us = 0);
#endif // CONFIG_BOARDCTL_RESET


/**
 * Request the system to shut down or reboot.
 * Note the following:
 * - The system might not support shutdown. In that case -EINVAL will
 *   be returned.
 * - The system might not shutdown immediately, so expect this method to return even
 *   on success.
 * @param delay_us optional delay in microseconds
 * @return 0 on success, <0 on error
 */
#if defined(BOARD_HAS_POWER_CONTROL) || defined(__PX4_POSIX)
__EXPORT int px4_shutdown_request(uint32_t delay_us = 0);
#endif // BOARD_HAS_POWER_CONTROL

/**
 * Initialize shutdown uORB mechanism.
 */
void shutdown_init();

__END_DECLS

class ShutdownEventCallback : public uORB::SubscriptionCallback
{
public:
	ShutdownEventCallback(void (*callback)(void *), void *parent) :
		uORB::SubscriptionCallback(ORB_ID(shutdown_event)),
		_callback(callback), _parent(parent)
	{
		if (_callback && _parent) {
			registerCallback();
			_shutdown_handle = px4_register_shutdown_hook();
		}
	}

	~ShutdownEventCallback()
	{
		if (_shutdown_handle >= 0) {
			px4_unregister_shutdown_hook(_shutdown_handle);
			unregisterCallback();
		}
	}

	void call()
	{
		shutdown_event_s msg;

		if (update(&msg)) {
			if (_callback && _parent) {
				_callback(_parent);
			}
		}
	}

	/**
	 * Confirm that subscriber has completed reboot/shutdown related procedures.
	 */
	void complete()
	{
		if (px4_unregister_shutdown_hook(_shutdown_handle) == PX4_OK) {
			unregisterCallback();
			_shutdown_handle = -1;
		}
	}
private:
	shutdown_handle_t _shutdown_handle{-1};
	void (*_callback)(void *) {nullptr};
	void *_parent{nullptr};
};
