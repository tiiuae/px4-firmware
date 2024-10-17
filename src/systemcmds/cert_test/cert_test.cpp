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

struct thread_test_info {
	BgProcExec *actuator;
	uint32_t cansend_status;
};

static struct thread_test_info thread_test;

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

static int setup_params()
{
	PX4_INFO("Setup device params.");

	const char *cmd_argv[] = {0, "set", "SYS_AUTOSTART", "4400", nullptr};

	BgProcExec cmd {"param set", "param", cmd_argv, false};
	px4_sleep(1);

	cmd_argv[2] = "SENS_EN_SDP3X";
	cmd_argv[3] = "1";

	cmd = BgProcExec {"param set", "param", cmd_argv, false};
	px4_sleep(1);

	cmd_argv[2] = "SDLOG_MODE";
	cmd_argv[3] = "2";

	cmd = BgProcExec {"param set", "param", cmd_argv, false};
	px4_sleep(1);

	cmd_argv[2] = "SDLOG_ALGORITHM";
	cmd_argv[3] = "0";

	cmd = BgProcExec {"param set", "param", cmd_argv, false};
	px4_sleep(1);

	px4_sleep(2);
	PX4_INFO("Reboting...");

	cmd_argv[1] = "-b";
	cmd_argv[2] = "-c";
	cmd_argv[3] = nullptr;
	cmd = BgProcExec {"reboot", "reboot", cmd_argv, false};

	return 0;
}

static void* can_test_thread(void *args)
{
	PX4_INFO("can test thread started");

	const char *cmd_argv1[] = {0, "can0", "123#10101010",  nullptr};
	BgProcExec *can_test1 = new BgProcExec("cansend", "cansend", cmd_argv1, false);

	const char *cmd_argv2[] = {0, "can0", "123#5A5A5A5A", nullptr};
	BgProcExec *can_test2 = new BgProcExec("cansend", "cansend", cmd_argv2, false);

	while (!stop_test) {
		if (!can_test1->rerun(nullptr)) {
			if (!(thread_test.cansend_status & OrbBase::STATUS_NOT_RUNNING)) {
				PX4_ERR("cansend process creation failed");
			}
			thread_test.cansend_status = OrbBase::STATUS_NOT_RUNNING;
			continue;
		}

		if (!can_test2->rerun(nullptr)) {
			if (!(thread_test.cansend_status & OrbBase::STATUS_NOT_RUNNING)) {
				PX4_ERR("cansend process creation failed");
			}
			thread_test.cansend_status = OrbBase::STATUS_NOT_RUNNING;
			continue;
		}

		thread_test.cansend_status = OrbBase::STATUS_OK;
	}

	thread_test.cansend_status = OrbBase::STATUS_INIT;

	delete can_test1;
	delete can_test2;

	return NULL;
}

static int start_can_test(pthread_t *can_thread)
{
	pthread_attr_t th_attr;

	thread_test.cansend_status = OrbBase::STATUS_INIT;

	if (netlib_ifup("can0") == -1) {
		PX4_ERR("netlib_ifup can0");
		return 1;
	} else {
		PX4_INFO("Enable can0");
	}

	if (pthread_attr_init(&th_attr)) {
		PX4_ERR("pthread_attr_init");
		return 1;
	}

	if (pthread_attr_setstacksize(&th_attr, 4096)) {
		PX4_ERR("pthread_attr_setstacksize");
		return 1;
	}

	if (pthread_create(can_thread, &th_attr, can_test_thread, nullptr))
	{
		PX4_ERR("pthread_create: %d", errno);
		return 1;
	}

	return 0;
}

static void start_actuator_test()
{
	const char *cmd_argv[] = {0, "set", "-m", "1", "-v", "0.25", nullptr};

	if (thread_test.actuator != nullptr) {
		if (thread_test.actuator->rerun(cmd_argv)) {
			PX4_INFO("actuator test re-launched");
		} else {
			PX4_ERR("actuator test re-launch failed");
		}
	} else {
		thread_test.actuator = new BgProcExec("actuator_test", "actuator_test", cmd_argv, true);
	}
}

static BgProcExec* start_telem_test()
{
	const char *cmd_argv[] = {0, "-t", "uart_chain_loopback", "/dev/ttyS1", "/dev/ttyS3", "/dev/ttyS4", nullptr};

	BgProcExec *telem_test = new BgProcExec("telem_test", "telem_test", cmd_argv, true);
	return telem_test;
}

static void stop_mavlink_uart()
{
	const char *cmd_argv[] = {0, "stop", "-d", "/dev/ttyS1", nullptr};

	BgProcExec *mavlink_uart = new BgProcExec("stop mavlink uart", "mavlink", cmd_argv, false);
	px4_sleep(3);
	delete mavlink_uart;
}

static int cert_test_task(int argc, char **argv)
{
	bool verbose = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	pthread_t can_thread = -1;

	PX4_INFO("cert_test starting");

	if (setup_sigaction()) {
		return 1;
	}

	while ((ch = px4_getopt(argc, argv, "vs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'v':
			verbose = true;
			break;

		case 's':
			return setup_params();

		default:
			PX4_ERR("Unknown option");
		}
	}

	start_actuator_test();

	stop_mavlink_uart();

	BgProcExec *telem_test = start_telem_test();

	if (start_can_test(&can_thread)) {
		stop_test = 1;
	}

	px4_sleep(10);

	PX4_INFO("start status reporting");

	CertTestStatus *cert_test = new CertTestStatus(thread_test.actuator, thread_test.cansend_status, verbose);

	if (!thread_test.actuator ||
		!telem_test ||
		!cert_test) {
		stop_test = 1;
	}

	while(!stop_test) {
		cert_test->update();

		if (thread_test.actuator->get_pid(false) < 0) {
			PX4_INFO("Try to re-launch actuator test...");
			start_actuator_test();
		}

		px4_sleep(1);
	}

	if (pthread_join(can_thread, NULL))
	{
		PX4_ERR("can_thread join: %d", errno);
	}

	delete thread_test.actuator;
	delete telem_test;
	delete cert_test;

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
