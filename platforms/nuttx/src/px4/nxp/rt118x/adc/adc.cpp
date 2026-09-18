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

/*
 * This is a placeholder LPADC arch implementation for i.MX RT1180 (rt118x).
 * The register level driver has not been ported/validated for this chip
 * yet, so all entry points are stubbed out to keep the build working for
 * boards that enable CONFIG_DRIVERS_ADC_BOARD_ADC without any ADC channels
 * configured. Replace with a real implementation once the ADC is wired up.
 */

#include <board_config.h>
#include <stdint.h>
#include <errno.h>

#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>

__EXPORT uint32_t px4_arch_adc_temp_sensor_mask(void)
{
	return 0;
}

__EXPORT int px4_arch_adc_init(uint32_t base_address)
{
	return -ENODEV;
}

__EXPORT void px4_arch_adc_uninit(uint32_t base_address)
{
}

__EXPORT uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	return UINT32_MAX;
}

__EXPORT float px4_arch_adc_reference_v(void)
{
	return 3.3f;
}

__EXPORT uint32_t px4_arch_adc_dn_fullcount(void)
{
	return 1 << 12;
}
