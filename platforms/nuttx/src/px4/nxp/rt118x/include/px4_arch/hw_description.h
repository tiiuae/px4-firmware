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

#pragma once


#include <stdint.h>

#include "hardware/imxrt_flexpwm.h"

#include <px4_platform_common/constexpr_util.h>

#include <board_config.h>
#if !defined(CONFIG_ARCH_CHIP_MIMXRT1189CVM8C)
# error "This code has only been validated with IMXRT1189. Make sure it is correct before using it on another board."
#endif

/*
 * i.MX RT1180 (IOMUX_VER3/RGPIO) packs the pad mux, pad electrical settings
 * and GPIO port/pin into a single 64-bit imxrt_pinset_t (see
 * imxrt_iomuxc_ver3.h and imxrt_rgpio.h), unlike older imxrt chips where a
 * GPIO pin is fully described by a 32-bit GPIO_PORTn | GPIO_PINn value.
 * There is no way to reconstruct the pad mux from a bare GPIO port/pin pair
 * (the pad index isn't a function of the port/pin alone), so this chip
 * does not use the GPIO::Port/GPIO::Pin abstraction that other imxrt chips
 * use for SPI chip-select/DRDY pins: boards instead define the complete,
 * ready-to-use pinset value with IOMUX_GPIO() in their own board_config.h
 * (the same way they already do for LEDs and buttons), and pass that value
 * directly to initSPIConfigExternal()/initSPIDevice() below. That value's
 * type, px4_gpio_pinset_t, is defined in this chip's micro_hal.h.
 */

/*
 * PWM
 *
 * i.MX RT1180 implements FlexPWM1..4 (4 modules of 4 submodules each),
 * matching RT1170. Only FlexPWM1, submodules 0-2 (PWM_A) are routed to the
 * imxrt1180-evk board's Arduino connector so far (see board_config.h); the
 * rest of the muxing is provided for completeness but is not yet validated
 * against real hardware.
 */

namespace PWM
{
enum FlexPWM {
	FlexPWM1 = 0,
	FlexPWM2,
	FlexPWM3,
	FlexPWM4,
};

enum FlexPWMModule {
	PWM1_PWM_A = 0,
	PWM1_PWM_B,
	PWM1_PWM_X,

	PWM2_PWM_A,
	PWM2_PWM_B,
	PWM2_PWM_X,

	PWM3_PWM_A,
	PWM3_PWM_B,
	PWM3_PWM_X,

	PWM4_PWM_A,
	PWM4_PWM_B,
	PWM4_PWM_X,
};

enum FlexPWMSubmodule {
	Submodule0 = 0,
	Submodule1,
	Submodule2,
	Submodule3,
};

struct FlexPWMConfig {
	FlexPWMModule module;
	FlexPWMSubmodule submodule;
};
}

static inline constexpr uint32_t getFlexPWMBaseRegister(PWM::FlexPWM pwm)
{
	switch (pwm) {
	case PWM::FlexPWM1: return IMXRT_FLEXPWM1_BASE;

	case PWM::FlexPWM2: return IMXRT_FLEXPWM2_BASE;

	case PWM::FlexPWM3: return IMXRT_FLEXPWM3_BASE;

	case PWM::FlexPWM4: return IMXRT_FLEXPWM4_BASE;
	}

	return 0;
}

/*
 * SPI
 */

namespace SPI
{

enum class Bus {
	LPSPI1 = 1,
	LPSPI2,
	LPSPI3,
	LPSPI4,
	LPSPI5,
	LPSPI6,
};

/* See the comment above: CS/DRDY pins are a complete pinset value (built
 * with IOMUX_GPIO() in board_config.h), not a {port, pin} pair.
 */
using CS = px4_gpio_pinset_t;
using DRDY = px4_gpio_pinset_t;

struct bus_device_external_cfg_t {
	CS cs_gpio;
	DRDY drdy_gpio;
};

} // namespace SPI
