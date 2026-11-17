/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file KeystoreZeroizeTrigger.hpp
 *
 * PROOF OF CONCEPT.
 *
 * Two parts:
 *  - Zeroize(): the mechanism. Erases the NOR keystore, verifies the erase and
 *    resets the SoC. It has NO conditions of its own (armed state, flight
 *    state, critical activities): whatever trigger calls it decides.
 *  - The trigger: in this POC a button on one GPIO input (the same pad as the
 *    log erase trigger POC), held high for kHoldSamples polls. To be replaced
 *    by the real trigger later (tamper switch, vibration, command, ...).
 *
 * Without the keystore the bootloader cannot verify the PX4 image (TOC/BOOT
 * verification key, slot 1) and stays in the bootloader, so the vehicle no
 * longer boots.
 *
 * Recovery is only possible by re-flashing a keystore backup taken from the
 * same board (keys are wrapped with the chip unique key), e.g. over UUU.
 *
 * Runs kernel side in the protected build: it touches the pad mux and the NOR
 * flash directly.
 */

#pragma once

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#ifndef GPIO_ZEROIZE_TRIGGER
#  error "board needs to define a keystore zeroize trigger gpio pin to use this module"
#endif

class KeystoreZeroizeTrigger : public ModuleBase<KeystoreZeroizeTrigger>, public px4::ScheduledWorkItem
{
public:
	KeystoreZeroizeTrigger();
	~KeystoreZeroizeTrigger() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	int Start();

	/** Apply the pad mux and configure the pin as an input. Safe to call more than once. */
	static int ConfigurePin();

	/** One-shot: configure, sample once, print. Backs the `read` command. Never erases. */
	static int ReadOnce();

	/**
	 * Read the keystore header and the key slots from NOR and print which ones
	 * are populated. Read-only. Backs the `keystore` command.
	 *
	 * @return number of populated slots, or a negative errno
	 */
	static int InspectKeystore(bool verbose);

private:
	void Run() override;

	/** Feed one raw sample into the debouncer; true when the debounced level changed. */
	bool UpdateDebouncedLevel(bool raw);

	/**
	 * The zeroize mechanism: erase the keystore, verify it (up to kEraseAttempts
	 * tries), then reset the SoC into the bootloader. Unconditional: callers are
	 * the triggers and they alone decide when to call it. Only returns if the
	 * erase could not be verified.
	 *
	 * @param reason what triggered it, for the log
	 */
	void Zeroize(const char *reason);

	/** One erase pass followed by a read-back check. */
	bool EraseAndVerify();

	static constexpr uint32_t kPollIntervalUs{20000}; ///< 20 ms == 50 Hz

	/** Consecutive agreeing samples before a level is believed: 3 at 50 Hz = 60 ms. */
	static constexpr uint8_t kDebounceSamples{3};

	/** Debounced-high samples required to fire: 250 at 50 Hz = 5 s. */
	static constexpr uint16_t kHoldSamples{250};

	/** Erase + verify passes before giving up. */
	static constexpr int kEraseAttempts{3};

	/** Delay between a verified erase and the reset, so the console message gets out. */
	static constexpr uint32_t kResetDelayUs{100000};

	// --- debounce state ---
	bool _raw_prev{false};
	uint8_t _stable_count{0};
	bool _level{false};
	bool _have_level{false};

	// --- hold / latch state ---
	uint16_t _hold_count{0};
	bool _latched{false};
	uint8_t _last_progress_s{0};

	// --- diagnostics ---
	uint32_t _edges{0};
	const char *_last_result{"none yet"};
};
