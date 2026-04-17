/****************************************************************************
 *
 * Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_dshot.h>

#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/timers/dshot.h>

#include <drivers/drv_hrt.h>

#ifndef PX4_DSHOT_DEVICE_PATH
#define PX4_DSHOT_DEVICE_PATH "/dev/dshot0"
#endif

#ifndef MODULE_NAME
#define MODULE_NAME "imx9_dshot"
#endif

static int g_dshot_fd = -1;
static bool g_bidirectional_enabled = false;
static struct dshot_throttle_s g_throttle;

static int bdshot_channel_online_status(uint8_t channel)
{
	if (!g_bidirectional_enabled || channel >= _DSHOT_NCHANNELS) {
		return -1;
	}

	if ((g_throttle.telemetry_req & (1u << channel)) == 0) {
		return -1;
	}

	/* Here we assume that the time base for channels is the same as for hrt;
	 * we have ensured this by defining our own board_get_systime
	 * for the nuttx DShot driver
	 */

	hrt_abstime timestamp = ts_to_abstime(&g_throttle.ch_telemetry[channel].timestamp);
	const int age_us = hrt_elapsed_time(&timestamp);
	return (age_us >= 0 && age_us < 100000) ? 1 : 0;
}

static inline uint16_t limit_channel_mask(uint32_t channel_mask)
{
	const uint32_t max_mask = (uint32_t)((1u << _DSHOT_NCHANNELS) - 1u);
	return (uint16_t)(channel_mask & max_mask);
}

int up_dshot_init(uint32_t channel_mask, unsigned dshot_freq, unsigned dshot_tlm_freq, bool enable_bidirectional_dshot)
{
	if (g_dshot_fd < 0) {
		g_dshot_fd = open(PX4_DSHOT_DEVICE_PATH, O_RDONLY);

		if (g_dshot_fd < 0) {
			return g_dshot_fd;
		}
	}

	struct dshot_config_s config;

	memset(&config, 0, sizeof(config));

	config.freq = dshot_freq;

	config.telem_freq = dshot_tlm_freq;

	config.bidir = enable_bidirectional_dshot;

	config.active_mask = limit_channel_mask(channel_mask);

	if (config.active_mask == 0) {
		return -EINVAL;
	}

	PX4_DEBUG("mask %x, bidir %d, telem_freq %d, freq %d", config.active_mask, config.bidir, config.telem_freq,
		  config.freq);
	int ret = ioctl(g_dshot_fd, DSHOTIOC_CONFIGURE, (unsigned long)(uintptr_t)&config);

	if (ret < 0) {
		return ret;
	}

	g_throttle.ch_mask = config.active_mask;
	g_throttle.telemetry_req = 0;
	g_bidirectional_enabled = enable_bidirectional_dshot;

	return g_throttle.ch_mask;
}

void dshot_motor_data_set(unsigned motor_number, uint16_t throttle, bool telemetry)
{
	if (motor_number >= _DSHOT_NCHANNELS || (g_throttle.ch_mask & (1u << motor_number)) == 0) {
		return;
	}

	g_throttle.throttle[motor_number] = throttle;

	if (telemetry) {
		g_throttle.telemetry_req |= (1u << motor_number);

	} else {
		g_throttle.telemetry_req &= (uint16_t)~(1u << motor_number);
	}
}

void up_dshot_trigger(void)
{
	if (g_dshot_fd < 0 || g_throttle.ch_mask == 0) {
		return;
	}

	if (ioctl(g_dshot_fd, DSHOTIOC_SET_THROTTLE, (unsigned long)(uintptr_t)&g_throttle) < 0) {
		return;
	}
}

int up_dshot_arm(bool armed)
{
	if (!armed) {
		for (unsigned i = 0; i < _DSHOT_NCHANNELS; ++i) {
			g_throttle.throttle[i] = 0;
		}
	}

	up_dshot_trigger();

	return 0;
}

void up_bdshot_status(void)
{
	printf("Bidirectional telemetry %s, channel mask %x\n",
	       g_bidirectional_enabled ? "enabled" : "disabled",
	       g_throttle.ch_mask);
}

uint16_t up_bdshot_get_ready_mask(void)
{
	if (!g_bidirectional_enabled) {
		return 0;
	}

	uint16_t ready_mask = 0;

	for (unsigned i = 0; i < _DSHOT_NCHANNELS; ++i) {
		if ((g_throttle.ch_mask & (1u << i)) != 0 && bdshot_channel_online_status(i) > 0) {
			ready_mask |= (1u << i);
		}
	}

	return ready_mask;
}

int up_bdshot_num_errors(uint8_t channel)
{
	if (!g_bidirectional_enabled || channel >= _DSHOT_NCHANNELS) {
		return -1;
	}

	/* NuttX wrapper does not expose telemetry decode error counters yet. */
	return 0;
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	if (!erpm) {
		return -EINVAL;
	}

	if (bdshot_channel_online_status(channel) <= 0) {
		return -1;
	}

	*erpm = g_throttle.ch_telemetry[channel].erpm;
	return 0;
}

int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value)
{
	if (!value) {
		return -EINVAL;
	}

	if (bdshot_channel_online_status(channel) <= 0) {
		return -1;
	}

	if (g_throttle.ch_telemetry[channel].edt_type != type) {
		return -EAGAIN;
	}

	*value = g_throttle.ch_telemetry[channel].edt_value;
	return 0;
}

int up_bdshot_channel_online(uint8_t channel)
{
	return bdshot_channel_online_status(channel);
}
