/*
 * Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 */

/*
 * FlexPWM channel setup for i.MX RT1180 (rt118x).
 *
 * Unlike boards/px4/fmu-v6xrt (imxrt1170/rt117x), which matches a
 * IOMUX::Pad enum against a big per-chip pad/submodule switch to build
 * each channel's GPIO pinset, i.MX RT1180 uses the IOMUX_VER3/RGPIO pad
 * scheme (see hw_description.h): a GPIO pin is a single, complete 64-bit
 * pinset that can't be derived from a pad enum plus generic flags. So
 * instead, the board builds both the peripheral and GPIO-output pinsets
 * for each actuator pin in its own board_config.h and passes them directly
 * to initIOTimerChannel() below.
 */

#pragma once

#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>

#include <hardware/imxrt_flexpwm.h>

#include <board_config.h>
#if !defined(CONFIG_ARCH_CHIP_MIMXRT1189CVM8C)
# error "This code has only been validated with IMXRT1189. Make sure it is correct before using it on another board."
#endif

static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		PWM::FlexPWMConfig pwm_config, px4_gpio_pinset_t gpio_out, px4_gpio_pinset_t gpio_portpin)
{
	timer_io_channels_t ret{};
	PWM::FlexPWM pwm {};

	ret.gpio_out = gpio_out;
	ret.gpio_in = 0; // Capture is not used
	ret.gpio_portpin = gpio_portpin;

	constexpr_assert(ret.gpio_out != 0, "Invalid PWM pin config");
	constexpr_assert(ret.gpio_portpin != 0, "Invalid GPIO output pin config");

	switch (pwm_config.module) {
	case PWM::PWM1_PWM_A:
		pwm = PWM::FlexPWM1;
		ret.val_offset = PWMA_VAL;
		break;

	case PWM::PWM1_PWM_B:
		pwm = PWM::FlexPWM1;
		ret.val_offset = PWMB_VAL;
		break;

	case PWM::PWM1_PWM_X:
		pwm = PWM::FlexPWM1;
		ret.val_offset = PWMX_VAL;
		break;

	case PWM::PWM2_PWM_A:
		pwm = PWM::FlexPWM2;
		ret.val_offset = PWMA_VAL;
		break;

	case PWM::PWM2_PWM_B:
		pwm = PWM::FlexPWM2;
		ret.val_offset = PWMB_VAL;
		break;

	case PWM::PWM2_PWM_X:
		pwm = PWM::FlexPWM2;
		ret.val_offset = PWMX_VAL;
		break;

	case PWM::PWM3_PWM_A:
		pwm = PWM::FlexPWM3;
		ret.val_offset = PWMA_VAL;
		break;

	case PWM::PWM3_PWM_B:
		pwm = PWM::FlexPWM3;
		ret.val_offset = PWMB_VAL;
		break;

	case PWM::PWM3_PWM_X:
		pwm = PWM::FlexPWM3;
		ret.val_offset = PWMX_VAL;
		break;

	case PWM::PWM4_PWM_A:
		pwm = PWM::FlexPWM4;
		ret.val_offset = PWMA_VAL;
		break;

	case PWM::PWM4_PWM_B:
		pwm = PWM::FlexPWM4;
		ret.val_offset = PWMB_VAL;
		break;

	case PWM::PWM4_PWM_X:
		pwm = PWM::FlexPWM4;
		ret.val_offset = PWMX_VAL;
		break;

	default:
		constexpr_assert(false, "not implemented");
	}

	switch (pwm_config.submodule) {
	case PWM::Submodule0:
		ret.sub_module = SM0;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM0);
		break;

	case PWM::Submodule1:
		ret.sub_module = SM1;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM1);
		break;

	case PWM::Submodule2:
		ret.sub_module = SM2;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM2);
		break;

	case PWM::Submodule3:
		ret.sub_module = SM3;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM3);
		break;
	}

	// find timer index
	ret.timer_index = 0xff;
	const uint32_t timer_base = getFlexPWMBaseRegister(pwm);

	for (int i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == timer_base && io_timers_conf[i].submodle == ret.sub_module) {
			ret.timer_index = i;
			break;
		}
	}

	constexpr_assert(ret.timer_index != 0xff, "Timer not found");

	return ret;
}

static inline constexpr io_timers_t initIOPWM(PWM::FlexPWM pwm, PWM::FlexPWMSubmodule sub)
{
	io_timers_t ret{};

	ret.base = getFlexPWMBaseRegister(pwm);
	ret.submodle = sub;
	return ret;
}

static inline constexpr io_timers_t initIOPWMDshot(PWM::FlexPWM pwm, PWM::FlexPWMSubmodule sub, uint32_t pinmux,
		uint32_t flexio_pin)
{
	io_timers_t ret{};

	ret.base = getFlexPWMBaseRegister(pwm);
	ret.submodle = sub;
	ret.dshot.pinmux = pinmux;
	ret.dshot.flexio_pin = flexio_pin;
	return ret;
}
