/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file LogEraseTrigger.cpp
 *
 * BRING-UP PROTOTYPE: read one GPIO, print every edge. See LogEraseTrigger.hpp.
 */

#include <board_config.h>
#include "LogEraseTrigger.hpp"

#include <lib/log_utils/log_utils.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <errno.h>

LogEraseTrigger::LogEraseTrigger() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

LogEraseTrigger::~LogEraseTrigger()
{
	ScheduleClear();
}

int
LogEraseTrigger::ConfigurePin()
{
	int ret = imx9_iomux_configure(GPIO_LOG_ERASE_TRIGGER_MUX);

	if (ret < 0) {
		PX4_ERR("imx9_iomux_configure(%s) failed: %d", GPIO_LOG_ERASE_TRIGGER_NAME, ret);
		return ret;
	}

	ret = px4_arch_configgpio(GPIO_LOG_ERASE_TRIGGER);

	if (ret < 0) {
		PX4_ERR("px4_arch_configgpio(%s) failed: %d", GPIO_LOG_ERASE_TRIGGER_NAME, ret);
		return ret;
	}

	return PX4_OK;
}

int
LogEraseTrigger::ReadOnce()
{
	int ret = ConfigurePin();

	if (ret != PX4_OK) {
		return ret;
	}

	// Let the pin settle before believing it.
	px4_usleep(1000);

	const bool level = px4_arch_gpioread(GPIO_LOG_ERASE_TRIGGER);

	PX4_INFO("[GPIO %s Received at %s]", level ? "HIGH" : "LOW ", GPIO_LOG_ERASE_TRIGGER_NAME);

	return PX4_OK;
}

bool
LogEraseTrigger::UpdateDebouncedLevel(bool raw)
{
	if (raw != _raw_prev) {
		// Level moved: restart the agreement count. Bounce lands here.
		_raw_prev = raw;
		_stable_count = 0;
		return false;
	}

	if (_stable_count < kDebounceSamples) {
		_stable_count++;
	}

	if (_stable_count < kDebounceSamples) {
		return false;
	}

	if (_have_level && raw == _level) {
		return false;
	}

	_level = raw;
	_have_level = true;

	return true;
}

bool
LogEraseTrigger::GuardsPass(const char *&reason)
{
	if (_armed) {
		reason = "vehicle is armed";
		return false;
	}

	if (_logger_last_seen != 0 && hrt_elapsed_time(&_logger_last_seen) < kLoggerIdleTimeoutUs) {
		reason = "logger is writing";
		return false;
	}

	return true;
}

void
LogEraseTrigger::PerformErase()
{
	_triggers++;

	PX4_INFO("trigger held %.1f s on %s: erasing logs in %s",
		 (double)(kHoldSamples * kPollIntervalUs) / 1e6,
		 GPIO_LOG_ERASE_TRIGGER_NAME, log_utils::log_root());

	const int ret = log_utils::erase_all_logs();

	if (ret < 0) {
		_last_result = "failed";
		PX4_ERR("erase failed: %d", ret);

	} else {
		_last_result = "ok";
		PX4_INFO("erase complete: %d entries removed", ret);
	}
}

void
LogEraseTrigger::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// --- interlock inputs ---
	actuator_armed_s armed;

	if (_armed_sub.update(&armed)) {
		_armed = armed.armed;
	}

	for (auto &sub : _logger_status_sub) {
		logger_status_s status;

		if (sub.update(&status)) {
			_logger_last_seen = hrt_absolute_time();
		}
	}

	// --- debounced level ---
	const bool raw = px4_arch_gpioread(GPIO_LOG_ERASE_TRIGGER);
	const bool had_level = _have_level;

	if (UpdateDebouncedLevel(raw)) {
		PX4_INFO("[GPIO %s Received at %s]", _level ? "HIGH" : "LOW ", GPIO_LOG_ERASE_TRIGGER_NAME);

		if (had_level) {
			// The very first accepted reading is the baseline, not an edge.
			_edges++;
		}
	}

	if (!_level) {
		// Released: re-arm the trigger and allow a fresh refusal message.
		_hold_count = 0;
		_latched = false;
		_refusal_reported = false;
		return;
	}

	if (_latched) {
		// Already fired for this press; wait for release.
		return;
	}

	const char *reason = nullptr;

	if (!GuardsPass(reason)) {
		if (_hold_count > 0 || !_refusal_reported) {
			PX4_WARN("erase trigger held but refused: %s", reason);
			_refusal_reported = true;
			_refusals++;
		}

		_hold_count = 0;
		return;
	}

	_hold_count++;

	if (_hold_count >= kHoldSamples) {
		_latched = true;
		_hold_count = 0;
		PerformErase();
	}
}

int
LogEraseTrigger::Start()
{
	int ret = ConfigurePin();

	if (ret != PX4_OK) {
		return ret;
	}

	ScheduleOnInterval(kPollIntervalUs);

	return PX4_OK;
}

int
LogEraseTrigger::task_spawn(int argc, char *argv[])
{
	LogEraseTrigger *instance = new LogEraseTrigger();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -ENOMEM;
	}

	int ret = instance->Start();

	if (ret != PX4_OK) {
		delete instance;
		return ret;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return ret;
}

int
LogEraseTrigger::print_status()
{
	PX4_INFO("polling %s at %lu Hz, hold %.1f s to fire", GPIO_LOG_ERASE_TRIGGER_NAME,
		 (unsigned long)(1000000 / kPollIntervalUs),
		 (double)(kHoldSamples * kPollIntervalUs) / 1e6);

	if (_have_level) {
		PX4_INFO("level: %s, edges seen: %lu", _level ? "HIGH" : "LOW", (unsigned long)_edges);

	} else {
		PX4_INFO("level: not sampled yet");
	}

	if (_latched) {
		PX4_INFO("hold: latched, waiting for release");

	} else if (_hold_count > 0) {
		PX4_INFO("hold: %.1f / %.1f s",
			 (double)(_hold_count * kPollIntervalUs) / 1e6,
			 (double)(kHoldSamples * kPollIntervalUs) / 1e6);

	} else {
		PX4_INFO("hold: idle");
	}

	const char *reason = nullptr;
	const bool permitted = GuardsPass(reason);

	PX4_INFO("interlock: %s%s%s", permitted ? "clear" : "BLOCKED (", permitted ? "" : reason,
		 permitted ? "" : ")");
	PX4_INFO("  armed: %s", _armed ? "yes" : "no");

	if (_logger_last_seen == 0) {
		PX4_INFO("  logger: idle (no logger_status seen since start)");

	} else {
		PX4_INFO("  logger: %s (last logger_status %.1f s ago)",
			 hrt_elapsed_time(&_logger_last_seen) < kLoggerIdleTimeoutUs ? "WRITING" : "idle",
			 (double)hrt_elapsed_time(&_logger_last_seen) / 1e6);
	}

	PX4_INFO("erases: %lu, refusals: %lu, last result: %s",
		 (unsigned long)_triggers, (unsigned long)_refusals, _last_result);

	// Exercises the log_utils link: read-only, so it cannot disturb the logger.
	const int log_count = log_utils::count_logs();

	if (log_count < 0) {
		PX4_INFO("log dir %s: unavailable (%d)", log_utils::log_root(), log_count);

	} else {
		PX4_INFO("log dir %s: %d file(s)", log_utils::log_root(), log_count);
	}

	return PX4_OK;
}

int
LogEraseTrigger::custom_command(int argc, char *argv[])
{
	if (argc > 0 && !strcmp(argv[0], "read")) {
		return ReadOnce();
	}

	return print_usage("unknown command");
}

int
LogEraseTrigger::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Erases all onboard logs when a GPIO input is held.

The pin is TELEM1 CTS: pad GPIO_IO16 used as GPIO2_16, reached on base board
connector J13 pin 4 (3.3 V logic -- J13 pin 1 is 5 V, do not use it as a
source). Defined at compile time in LogEraseTrigger.hpp.

The input is polled at 50 Hz and debounced over 60 ms. Holding it high for 3 s
erases everything below /fs/microsd/log, then latches until the input is
released, so one press means at most one erase.

Two interlocks must hold for the entire 3 s, or the hold is cancelled:
 - the vehicle is disarmed
 - the logger is not writing, judged by logger_status having gone stale

Erasing is irreversible, and unrecoverable on boards that encrypt logs.

### Examples
Sample the pin once without starting the poller:
$ log_erase_trigger read

Show level, hold progress and interlock state:
$ log_erase_trigger status

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("log_erase_trigger", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("read", "configure the pin, sample it once, print the level");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int log_erase_trigger_main(int argc, char *argv[]);

int
log_erase_trigger_main(int argc, char *argv[])
{
	return LogEraseTrigger::main(argc, argv);
}
