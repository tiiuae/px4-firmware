/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file LogEraseTrigger.hpp
 *
 * BRING-UP PROTOTYPE.
 *
 * Polls one GPIO input and prints a line on every edge, so that a candidate
 * pad can be confirmed as readable on real hardware. It does not erase
 * anything and it has no debounce/hold logic yet -- that arrives once the pad
 * is confirmed.
 */

#pragma once

#include <board_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

/*****************************************************************************
 * TRIGGER PIN DEFINITION -- TELEM1 CTS
 *
 * Pad GPIO_IO16, used as GPIO2_16, reached externally on the TELEM1 connector.
 *
 *   SoC pad          GPIO_IO16               (i.MX93 GPIO2 bank, pin 16)
 *   FMU connector    J1 pin 24               (net UART7_CTS_TELEM1)
 *   base board       J13 pin 4 "TELEM1 CTS"  (BM06B-GHS-TBT, 6-pin JST GH)
 *                    J13: 1=VCC_5V, 2=TX, 3=RX, 4=CTS, 5=RTS, 6=GND
 *   conditioning     33R series + TPD4E6B06 TVS on the base board
 *
 * Chosen because it is an input by nature, is broken out on a standard GH
 * connector, and touches nothing safety critical -- unlike FMU_CAP1 (owned by
 * the redundancy armed watchdog, board_config.h:176) or the TELEM3 flow control
 * lines (which are the JTAG/SWD pads DAP_TCLK/DAP_TMS).
 *
 * Logic level is 3.3 V: this pad is in the same GPIO_IO group as LPUART4
 * TX/RX (GPIO_IO14/GPIO_IO15, board.h:73-74), which drive a telemetry radio
 * directly with no level shifter on the base board. Do NOT source the high
 * level from J13 pin 1 -- that is 5 V and would damage the pad.
 *
 * Ownership: this pad is LPUART4_CTS_B when TELEM1 output flow control is on.
 * NuttX muxes it at imx9_lowputc.c:208-210 under CONFIG_LPUART4_OFLOWCONTROL,
 * so that symbol is disabled in this board's defconfig and the #error below
 * keeps the two uses from silently fighting over the pad. TELEM1 keeps working;
 * it just stops honouring CTS from the radio, which is the usual PX4 setup.
 * CONFIG_LPUART4_IFLOWCONTROL is untouched -- that gates RTS (GPIO_IO17).
 *
 * TODO(move): move GPIO_LOG_ERASE_TRIGGER_MUX and GPIO_LOG_ERASE_TRIGGER into
 *   boards/ssrc/common/imx9_common/src/board_config.h
 * wrapped in  #ifdef CONFIG_MODULES_LOG_ERASE_TRIGGER  -- mirroring the
 * CONFIG_MODULES_REDUNDANCY_DRV block at board_config.h:169-180 -- and delete
 * them from here. They live in the module for now because board_config.h is
 * shared by every imx9 board and this is still a proof of concept.
 *
 * The pad macro and the pinset MUST agree:
 * IOMUXC_PAD_<pad>_GPIO<n>_IO<m> pairs with (GPIO_PORT<n> | GPIO_PIN<m>).
 * A mismatch reads a different pin and looks exactly like a wiring fault.
 *
 * Note: the pad mux is a separate step from the GPIO direction.
 * imx9_config_gpio() does not touch IOMUXC, so imx9_iomux_configure() must be
 * called too -- see init.c:226 for the safety button doing this.
 *****************************************************************************/
#if defined(CONFIG_LPUART4_OFLOWCONTROL)
#  error "log_erase_trigger uses GPIO_IO16, which TELEM1 (LPUART4) claims as CTS_B when CONFIG_LPUART4_OFLOWCONTROL is set. Disable that symbol or move the trigger to another pad."
#endif

#define GPIO_LOG_ERASE_TRIGGER_MUX \
	IOMUX_CFG(IOMUXC_PAD_GPIO_IO16_GPIO2_IO16, IOMUXC_PAD_PD_ON | IOMUXC_PAD_HYS_ST_ON, 0)
#define GPIO_LOG_ERASE_TRIGGER      (GPIO_PORT2 | GPIO_PIN16 | GPIO_INPUT)
#define GPIO_LOG_ERASE_TRIGGER_NAME "TELEM1 CTS (GPIO_IO16 / GPIO2_16, J13 pin 4)"
/*****************************************************************************/

class LogEraseTrigger : public ModuleBase<LogEraseTrigger>, public px4::ScheduledWorkItem
{
public:
	LogEraseTrigger();
	~LogEraseTrigger() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	int Start();

	/**
	 * Apply the pad mux and configure the pin as an input.
	 * Safe to call more than once.
	 *
	 * @return PX4_OK on success, negative errno otherwise
	 */
	static int ConfigurePin();

	/** One-shot: configure, sample once, print. Backs the `read` command. */
	static int ReadOnce();

private:
	void Run() override;

	static constexpr uint32_t kPollIntervalUs{20000}; ///< 20 ms == 50 Hz

	bool _state_prev{false};   ///< pin level on the previous poll
	bool _have_prev{false};    ///< false until the first sample, so the initial level is printed
	uint32_t _edges{0};        ///< edges seen since start, for print_status()
};
