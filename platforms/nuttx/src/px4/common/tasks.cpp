/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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

/**
 * @file tasks.cpp
 * Implementation of existing task API for NuttX
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#include <nuttx/board.h>
#include <nuttx/kthread.h>

#include <sys/wait.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <stdbool.h>

#ifdef CONFIG_BUILD_KERNEL

#ifndef MODULE_NAME
#define MODULE_NAME "PX4_TASKS"
#endif

// Adaptor code copy & pasted from posix/.../tasks.cpp
typedef struct {
	px4_main_t entry;
	char name[16]; //pthread_setname_np is restricted to 16 chars
	int argc;
	char *argv[];
	// strings are allocated after the struct data
} pthdata_t;

static void *entry_adapter(void *ptr)
{
	pthdata_t *data = (pthdata_t *) ptr;

	// set the threads name
	int rv = pthread_setname_np(pthread_self(), data->name);

	if (rv) {
		PX4_ERR("px4_task_spawn_cmd: failed to set name of thread %d %d\n", rv, errno);
	}

	data->entry(data->argc, data->argv);
	free(ptr);

	return NULL;
}

int task_create(const char *name, int priority,	int stack_size, main_t entry,
		char * const argv[])
{
	size_t stringssize;
	size_t structsize;

	pthread_t pid;
	pthread_attr_s attr = {
		.priority = (uint8_t)priority,
		.policy = SCHED_RR,
		.inheritsched = PTHREAD_EXPLICIT_SCHED,
		.detachstate = PTHREAD_CREATE_JOINABLE,
		.stackaddr = NULL,
		.stacksize = (size_t)stack_size,
	};

	int argc = 0;
	stringssize = strlen(name) + 1;

	// Count the arguments and size of the strings (with terminating NULL)
	if (argv) {
		while(argv[argc]) {
			stringssize += strlen(argv[argc]) + 1;
			if (stringssize >= (size_t)stack_size) {
				// Oops, we ran out of stack to copy the strings
				return -ENAMETOOLONG;
			}
			argc++;
		}
	}

	structsize = sizeof(pthdata_t) + (argc + 2) * sizeof(char *);

	// Not safe to pass stack data to the thread creation
	pthdata_t *taskdata = (pthdata_t *)malloc(structsize + stringssize);

	if (taskdata == NULL) {
		return -ENOMEM;
	}

	// Copy strings and pthread name to argv
	memset(taskdata, 0, structsize + stringssize);

	strncpy(taskdata->name, name, 16);
	taskdata->name[15] = '\0';
	taskdata->entry = entry;
	taskdata->argc = argc + 1;

	char *offset = (char *)taskdata + structsize;
	taskdata->argv[0] = offset;
	strcpy(offset, name);
	offset += strlen(name) + 1;

	for (int i = 0; i < argc; ++i) {
		taskdata->argv[i + 1] = offset;
		strcpy(offset, argv[i]);
		offset += strlen(argv[i]) + 1;
	}

	// Must add NULL at end of argv
	taskdata->argv[argc + 1] = (char *)NULL;

	if (pthread_create(&pid, &attr, &entry_adapter, taskdata) < 0) {
		// Pthread failed to start, clean up and get out
		free(taskdata);
		pid = ERROR;
	}

	pthread_attr_destroy(&attr);

	return (int)pid;
}
#endif

int px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, main_t entry, char *const argv[])
{
	sched_lock();

#if !defined(CONFIG_DISABLE_ENVIRON)
	/* None of the modules access the environment variables (via getenv() for instance), so delete them
	 * all. They are only used within the startup script, and NuttX automatically exports them to the children
	 * tasks.
	 * This frees up a considerable amount of RAM.
	 */
	clearenv();
#endif

#if !defined(__KERNEL__)
	/* create the task */
	int pid = task_create(name, priority, stack_size, entry, argv);
#else
	int pid = kthread_create(name, priority, stack_size, entry, argv);
#endif

	if (pid > 0) {
		/* configure the scheduler */
		struct sched_param param = { .sched_priority = priority };
		sched_setscheduler(pid, scheduler, &param);
	}

	sched_unlock();

	return pid;
}

int px4_task_delete(int pid)
{
	int ret = OK;

	if (pid == getpid()) {
		// Commit suicide
		exit(EXIT_SUCCESS);

	} else {
		// Politely ask someone else to kill themselves
		ret = kill(pid, SIGKILL);
	}

	return ret;
}

const char *px4_get_taskname(void)
{
	return getprogname();
}
