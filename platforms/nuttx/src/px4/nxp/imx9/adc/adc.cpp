/****************************************************************************
 *
 *   Copyright (C) 2024 Technology Innovation Institute. All rights reserved.
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

#include <board_config.h>
#include <errno.h>
#include <stdint.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <nuttx/analog/adc.h>
#include <px4_arch/adc.h>

#include <debug.h>

#include <imx9_adc.h>

static bool g_adc_configured = false;
static struct adc_dev_s *g_adc_dev = nullptr;


int px4_arch_adc_init(uint32_t base_address)
{
	int ret = -ENODEV;

	if (!g_adc_configured) {
		g_adc_dev = imx9_adc_initialize(base_address);

		if (g_adc_dev == nullptr) {
			return ret;
		}

		ret = ADC_SETUP(g_adc_dev);
		if (ret < 0) {
			g_adc_dev = nullptr;
			return ret;
		}

		g_adc_configured = true;
	}
	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	(void)base_address;

	if (g_adc_dev != nullptr) {
		ADC_SHUTDOWN(g_adc_dev);
		g_adc_dev = nullptr;
	}

	g_adc_configured = false;
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	(void)base_address;

	if (!g_adc_configured || g_adc_dev == nullptr) {
		return UINT32_MAX;
	}

	uint16_t result = 0;

	int ret = imx9_adc_read_channel(g_adc_dev,
			  static_cast<uint8_t>(channel), &result);

	if (ret < 0) {
		return UINT32_MAX;
	}

	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;
}

uint32_t px4_arch_adc_temp_sensor_mask()
{

	return 0;

}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1u << 12;
}
