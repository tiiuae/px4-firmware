/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "netutils/netlib.h"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>

#include "cert_test_status.hpp"

#include "orb_report.hpp"
#include "bg_proc.hpp"
#include "test_logger.hpp"
#include "can_test.hpp"

#include "test_config.hpp"

static TestLogger *logger;

static int stop_test = 0;

static void app_sighandler(int sig_num)
{
	stop_test = 1;
}

static int setup_sigaction(void)
{
	struct sigaction act = {0};
	act.sa_handler = app_sighandler;

	sigaddset(&act.sa_mask, SIGTERM);
	if (sigaction(SIGTERM, &act, NULL) != 0) {
		PX4_ERR("sigaction1: errno=%d", errno);
		return 1;
	}

	return 0;
}

struct param_set {
	uint32_t saluki_hw;
	const char *name;
	const char *value;
};

static struct param_set PARAM_SET_CFG[] = {
	{SALUKI_HW_ANY,		"SYS_AUTOSTART",		"4400"},
	{SALUKI_HW_ANY,		"SENS_EN_SDP3X",		"1"},
	{SALUKI_HW_ANY,		"SDLOG_MODE",			"2"},
	{SALUKI_HW_ANY,		"SDLOG_ALGORITHM",		"0"},
	{SALUKI_HW_FMU2,	"PWM_AUX_FUNC1",		"105"},
	{SALUKI_HW_ANY,		nullptr, nullptr},
};

static int setup_params(uint32_t saluki_hw)
{
	PX4_INFO("Setup device params.");

	TestLogger *px4log = new TestLogger();
	if (!px4log) {
		PX4_ERR("log creation failed");
		return 1;
	}
	px4log->init();

	const char *cmd_argv[] = {0, "set", "", "", nullptr};

	for (int i = 0; PARAM_SET_CFG[i].name != nullptr; i++) {
		if (PARAM_SET_CFG[i].saluki_hw != SALUKI_HW_ANY &&
			PARAM_SET_CFG[i].saluki_hw != saluki_hw) {
			continue;
		}

		cmd_argv[2] = PARAM_SET_CFG[i].name;
		cmd_argv[3] = PARAM_SET_CFG[i].value;

		PX4_INFO("Setting param %s to %s", cmd_argv[2], cmd_argv[3]);

		BgProcExec cmd {"param set", "param", cmd_argv, BgProcExec::NO_KILL, px4log};
		px4_sleep(1);
	}

	px4_sleep(2);
	PX4_INFO("Reboting...");

	cmd_argv[1] = "-b";
	cmd_argv[2] = "-c";
	cmd_argv[3] = nullptr;
	BgProcExec cmd {"reboot", "reboot", cmd_argv, BgProcExec::NO_KILL, px4log};

	delete px4log;

	return 0;
}

static BgProcExec* start_actuator_test(BgProcExec *actuator)
{
	const char *cmd_argv[] = {0, "set", "-m", "1", "-v", "0.25", nullptr};

	if (actuator == nullptr) {
		return new BgProcExec("actuator_test", "actuator_test", cmd_argv, BgProcExec::KILL, logger);
	}

	if (actuator->rerun(cmd_argv)) {
		logger->log(TestLogger::INFO, "actuator test re-launched");
	} else {
		logger->log(TestLogger::ERR, "actuator test re-launch failed");
	}

	return actuator;
}

static BgProcExec* start_telem_test()
{
	const char *cmd_argv[] = {0, "-t", "uart_chain_loopback", "/dev/ttyS1", "/dev/ttyS3", "/dev/ttyS4", nullptr};

	BgProcExec *telem_test = new BgProcExec("telem_test", "telem_test", cmd_argv, BgProcExec::KILL, logger);
	return telem_test;
}

static void stop_mavlink_uart()
{
	const char *cmd_argv[] = {0, "stop", "-d", "/dev/ttyS1", nullptr};

	BgProcExec *mavlink_uart = new BgProcExec("stop mavlink uart", "mavlink", cmd_argv, BgProcExec::NO_KILL, logger);
	px4_sleep(3);
	delete mavlink_uart;
}

static int cert_test_task(int argc, char **argv)
{
	bool verbose = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	uint32_t saluki_hw = SALUKI_HW_V2;
	bool run_setup = false;

	PX4_INFO("cert_test starting");

	if (setup_sigaction()) {
		return 1;
	}

	while ((ch = px4_getopt(argc, argv, "vsc:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'v':
			verbose = true;
			break;

		case 's':
			run_setup = true;
			break;

		case 'c':
			if (strcmp(myoptarg, "saluki_fmu2") == 0) {
				saluki_hw = SALUKI_HW_FMU2;
			}
			break;

		default:
			PX4_ERR("Unknown option");
		}
	}

	if (run_setup) {
		return setup_params(saluki_hw);
	}

	logger = new TestLogger("[cert_test]", "cert");
	if (!logger) {
		PX4_ERR("log creation failed");
		return 1;
	}

	if (logger->init()) {
		PX4_ERR("log init failed");
		return 1;
	}

	BgProcExec *actuator = start_actuator_test(nullptr);

	stop_mavlink_uart();

	BgProcExec *telem_test = start_telem_test();

	CanTest *can_test = new CanTest(logger);
	if (can_test->start()) {
		stop_test = 1;
	}

	px4_sleep(10);

	logger->log(TestLogger::INFO, "start status reporting");

	CertTestStatus *cert_test = new CertTestStatus(saluki_hw, actuator, can_test, logger, verbose);

	if (!actuator ||
		!telem_test ||
		!cert_test) {
		stop_test = 1;
		logger->log(TestLogger::ERR, "Test init failed [%p, %p, %p]", actuator, telem_test, cert_test);
	}

	while(!stop_test) {
		cert_test->update();

		if (actuator->get_pid(false) < 0) {
			logger->log(TestLogger::INFO, "Try to re-launch actuator test...");
			start_actuator_test(actuator);
		}

		px4_sleep(1);
	}

	logger->log(TestLogger::INFO, "shutting down");

	delete actuator;
	delete can_test;
	delete telem_test;
	delete cert_test;
	delete logger;

	PX4_INFO("cert test EXIT");

	return 0;
}

extern "C" __EXPORT int cert_test_main(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("cert_test_task",
					SCHED_DEFAULT,
					SCHED_PRIORITY_MAX - 5,
					4096,
					cert_test_task,
					(char* const*)argv);

	if (task_id < 1) {
		PX4_ERR("cert test start failed: %d", task_id);
		return 1;
	}

	return 0;
}
