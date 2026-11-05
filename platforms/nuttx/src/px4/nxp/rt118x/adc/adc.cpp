/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
#include <stdbool.h>
#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>

#include <hardware/imxrt_adc_ver3.h>
#include <hardware/imxrt_memorymap.h>
#include <imxrt_periphclks.h>

#define REG(base, offset) (*(volatile uint32_t *)((base) + (offset)))

#define ADC_CTRL(base)      REG(base, IMXRT_ADC_CTRL_OFFSET)
#define ADC_CFG(base)       REG(base, IMXRT_ADC_CFG_OFFSET)
#define ADC_SWTRIG(base)    REG(base, IMXRT_ADC_SWTRIG_OFFSET)
#define ADC_TCTRL0(base)    REG(base, IMXRT_ADC_TCTRL0_OFFSET)
#define ADC_FCTRL0(base)    REG(base, IMXRT_ADC_FCTRL0_OFFSET)
#define ADC_CMDL1(base)     REG(base, IMXRT_ADC_CMDL1_OFFSET)
#define ADC_CMDH1(base)     REG(base, IMXRT_ADC_CMDH1_OFFSET)
#define ADC_RESFIFO0(base)  REG(base, IMXRT_ADC_RESFIFO0_OFFSET)

#define ADC_SAMPLE_TIMEOUT_US 100

static bool g_adc1_initialized;
static bool g_adc2_initialized;

static bool adc_valid_base(uint32_t base_address)
{
	return base_address == IMXRT_ADC1_BASE || base_address == IMXRT_ADC2_BASE;
}

static void adc_clock_enable(uint32_t base_address)
{
	if (base_address == IMXRT_ADC1_BASE) {
		imxrt_clockall_adc1();

	} else {
		imxrt_clockall_adc2();
	}
}

static void adc_clock_disable(uint32_t base_address)
{
	if (base_address == IMXRT_ADC1_BASE) {
		imxrt_clockoff_adc1();

	} else {
		imxrt_clockoff_adc2();
	}
}

__EXPORT int px4_arch_adc_init(uint32_t base_address)
{
	if (!adc_valid_base(base_address)) {
		return -1;
	}

	bool *initialized = base_address == IMXRT_ADC1_BASE ?
			    &g_adc1_initialized : &g_adc2_initialized;

	if (*initialized) {
		return 0;
	}

	adc_clock_enable(base_address);
	ADC_CTRL(base_address) = IMXRT_ADC_CTRL_RST;
	ADC_CTRL(base_address) = 0;
	ADC_CTRL(base_address) = IMXRT_ADC_CTRL_RSTFIFO0;
	ADC_CTRL(base_address) = 0;
	ADC_CFG(base_address) = IMXRT_ADC_CFG_PWREN |
				(0x10u << IMXRT_ADC_CFG_PUDLY_SHIFT);
	ADC_CTRL(base_address) = IMXRT_ADC_CTRL_ADCEN;

	*initialized = true;
	return 0;
}

__EXPORT void px4_arch_adc_uninit(uint32_t base_address)
{
	if (!adc_valid_base(base_address)) {
		return;
	}

	ADC_CTRL(base_address) = 0;

	if (base_address == IMXRT_ADC1_BASE) {
		g_adc1_initialized = false;

	} else {
		g_adc2_initialized = false;
	}

	adc_clock_disable(base_address);
}

__EXPORT uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	if (!adc_valid_base(base_address) || channel > 15) {
		return UINT32_MAX;
	}

	ADC_CTRL(base_address) |= IMXRT_ADC_CTRL_RSTFIFO0;
	ADC_CTRL(base_address) &= ~IMXRT_ADC_CTRL_RSTFIFO0;
	unsigned hardware_channel = channel;
	unsigned channel_type = IMXRT_ADC_CMDL_CTYPE_A;

	/* PX4 channel IDs 0-7 are A0-A7; IDs 8-15 are B0-B7. */
	if (channel >= 8) {
		hardware_channel = channel - 8;
		channel_type = IMXRT_ADC_CMDL_CTYPE_B;
	}

	ADC_CMDL1(base_address) = IMXRT_ADC_CMDL_ADCH(hardware_channel) |
				  IMXRT_ADC_CMDL_CTYPE(channel_type) |
				  IMXRT_ADC_CMDL_CSCALE(1);
	ADC_CMDH1(base_address) = 0;
	ADC_TCTRL0(base_address) = IMXRT_ADC_TCTRL_TCMD(1);
	ADC_SWTRIG(base_address) = 1;

	const hrt_abstime start = hrt_absolute_time();

	while ((ADC_FCTRL0(base_address) & IMXRT_ADC_FCTRL_FCOUNT_MASK) == 0) {
		if (hrt_absolute_time() - start > ADC_SAMPLE_TIMEOUT_US) {
			return UINT32_MAX;
		}
	}

	return (ADC_RESFIFO0(base_address) & IMXRT_ADC_RESFIFO_DATA_MASK) >> 3;
}

__EXPORT float px4_arch_adc_reference_v(void)
{
#ifdef BOARD_ADC_POS_REF_V
	return BOARD_ADC_POS_REF_V;
#else
	return 3.3f;
#endif
}

__EXPORT uint32_t px4_arch_adc_temp_sensor_mask(void)
{
	return 0;
}

__EXPORT uint32_t px4_arch_adc_dn_fullcount(void)
{
	return 1 << 12;
}
