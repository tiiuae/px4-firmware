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
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/logger_status.h>

/*
 * The trigger pin is defined by the board, not by this module:
 *   boards/ssrc/common/imx9_common/src/board_config.h
 * under #ifdef CONFIG_MODULES_LOG_ERASE_TRIGGER, which is also where the pad
 * choice, its wiring down to the external connector, and the LPUART4 CTS
 * ownership conflict are documented.
 */
#ifndef GPIO_LOG_ERASE_TRIGGER
#  error "board needs to define a log erase trigger gpio pin to use this module"
#endif

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

	/**
	 * Feed one raw sample into the debouncer.
	 *
	 * @param raw the level just read from the pin
	 * @return true when the debounced level changed as a result
	 */
	bool UpdateDebouncedLevel(bool raw);

	/**
	 * Check the conditions that must hold before logs may be erased.
	 *
	 * @param reason set to a static string naming the blocking condition when
	 *               this returns false; untouched otherwise
	 * @return true when erasing is permitted
	 */
	bool GuardsPass(const char *&reason);

	/** Erase the logs and report the outcome. */
	void PerformErase();

	static constexpr uint32_t kPollIntervalUs{20000}; ///< 20 ms == 50 Hz

	/**
	 * Consecutive agreeing samples before a level is believed. 3 samples at
	 * 50 Hz is 60 ms, comfortably longer than the contact bounce seen when the
	 * input is driven by a switch or a hand-held jumper.
	 */
	static constexpr uint8_t kDebounceSamples{3};

	/**
	 * Debounced-high samples required to fire: 150 at 50 Hz is 3 s. Long
	 * enough that a glitch or a brief accidental contact cannot erase flight
	 * logs, short enough to be practical to hold deliberately.
	 */
	static constexpr uint16_t kHoldSamples{150};

	/**
	 * How stale logger_status must be before the logger counts as idle.
	 *
	 * Logger::publish_logger_status() (src/modules/logger/logger.cpp:1070-1099)
	 * publishes at 1 Hz and ONLY while _writer.is_started(), so the freshness
	 * of that topic is a direct proxy for "the writer has files open". 2.5 s
	 * tolerates scheduling jitter without being sluggish.
	 */
	static constexpr hrt_abstime kLoggerIdleTimeoutUs{2500000};

	/** logger_status is published multi-instance, one per LogType (full, mission). */
	static constexpr uint8_t kLoggerStatusInstances{2};

	// --- debounce state ---
	bool _raw_prev{false};       ///< previous raw sample, for the debouncer
	uint8_t _stable_count{0};    ///< how many consecutive samples have agreed
	bool _level{false};          ///< current debounced level
	bool _have_level{false};     ///< false until the first level is accepted

	// --- hold / latch state ---
	uint16_t _hold_count{0};      ///< consecutive debounced-high polls with guards passing
	bool _latched{false};         ///< fired already; wait for release before firing again
	bool _refusal_reported{false}; ///< keeps a blocked guard from spamming the console

	// --- interlock ---
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _logger_status_sub[kLoggerStatusInstances] {
		{ORB_ID(logger_status), 0},
		{ORB_ID(logger_status), 1},
	};
	hrt_abstime _logger_last_seen{0}; ///< last time any logger_status instance updated
	bool _armed{false};               ///< latest arming state

	// --- diagnostics, surfaced by print_status() ---
	uint32_t _edges{0};                     ///< debounced edges seen since start
	uint32_t _triggers{0};                  ///< times the hold completed and an erase ran
	uint32_t _refusals{0};                  ///< times a hold was blocked by a guard
	const char *_last_result{"none yet"};   ///< outcome of the most recent erase attempt
};
