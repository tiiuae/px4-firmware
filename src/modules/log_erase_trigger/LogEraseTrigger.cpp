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
	// The pad mux and the GPIO direction are two separate operations:
	// imx9_config_gpio() never touches IOMUXC, so without this the pin keeps
	// whatever function it had at reset and reads garbage.
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

void
LogEraseTrigger::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	const bool level = px4_arch_gpioread(GPIO_LOG_ERASE_TRIGGER);

	// Print on edges only. At 50 Hz, printing every poll while the pin is high
	// floods the console; and printing both edges (not just HIGH) is what
	// distinguishes a working input from a pad stuck high by a bad mux.
	if (!_have_prev || level != _state_prev) {
		PX4_INFO("[GPIO %s Received at %s]", level ? "HIGH" : "LOW ", GPIO_LOG_ERASE_TRIGGER_NAME);

		if (_have_prev) {
			_edges++;
		}

		_state_prev = level;
		_have_prev = true;
	}
}

int
LogEraseTrigger::Start()
{
	int ret = ConfigurePin();

	if (ret != PX4_OK) {
		// Refuse to run rather than poll a pin that was never muxed: a stream
		// of meaningless LOWs is worse than a clear failure.
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
	PX4_INFO("polling %s at %lu Hz", GPIO_LOG_ERASE_TRIGGER_NAME,
		 (unsigned long)(1000000 / kPollIntervalUs));

	if (_have_prev) {
		PX4_INFO("level: %s, edges seen: %lu", _state_prev ? "HIGH" : "LOW", (unsigned long)_edges);

	} else {
		PX4_INFO("level: not sampled yet");
	}

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
Bring-up prototype for the GPIO log-erase trigger.

Polls a single GPIO input and prints a line on every edge:

    [GPIO HIGH Received at TELEM1 CTS (GPIO_IO16 / GPIO2_16, J13 pin 4)]

The pin is TELEM1 CTS: pad GPIO_IO16 used as GPIO2_16, reached on base board
connector J13 pin 4 (3.3 V logic -- J13 pin 1 is 5 V, do not use it as a
source). Defined at compile time in LogEraseTrigger.hpp.

It does not erase logs: log_utils::erase_all_logs() is not implemented yet.

### Examples
Sample the pin once without starting the poller:
$ log_erase_trigger read

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
