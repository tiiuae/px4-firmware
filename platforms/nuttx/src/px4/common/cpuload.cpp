/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file cpuload.c
 *
 * Measurement of CPU load of each individual task.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 */
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform/cpuload.h>

#include <drivers/drv_hrt.h>

#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION)
__BEGIN_DECLS

#include <sys/syscall.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/note/note_driver.h>

__EXPORT struct system_load_s system_load;


void cpuload_start(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb);
void cpuload_stop(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb);
void cpuload_suspend(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb);
void cpuload_resume(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_cpuload_ops = {
	NULL,                  /* add */
	cpuload_start,         /* start */
	cpuload_stop,          /* stop */
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
	cpuload_suspend,       /* suspend */
	cpuload_resume,        /* resume */
#endif
#ifdef CONFIG_SMP
	NULL,                  /* cpu_start */
	NULL,                  /* cpu_started */
#  ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
	NULL,                  /* cpu_pause */
	NULL,                  /* cpu_paused */
	NULL,                  /* cpu_resume */
	NULL,                  /* cpu_resumed */
#  endif
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
	NULL,                  /* preemption */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
	NULL,                  /* csection */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
	NULL,                  /* spinlock */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
	NULL,                  /* syscall_enter */
	NULL,                  /* syscall_leave */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
	NULL,                  /* irqhandler */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct note_driver_s g_cpuload_driver = {
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
	"cpuload",
	{
		{
			CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE,
#  ifdef CONFIG_SMP
			CONFIG_SCHED_INSTRUMENTATION_CPUSET
#  endif
		},
	},
#endif
	&g_cpuload_ops,
};

/* Simple hashing via PID; shamelessly ripped from NuttX scheduler. All rights
 * and credit belong to whomever authored this logic.
 */

#define HASH(i) ((i) & (hashtab_size - 1))

struct system_load_taskinfo_s **hashtab;
volatile int hashtab_size;

void init_task_hash(void)
{
	hashtab_size = 4;
	hashtab = (struct system_load_taskinfo_s **)kmm_zalloc(sizeof(*hashtab) * hashtab_size);
}

static struct system_load_taskinfo_s *get_task_info(pid_t pid)
{
	struct system_load_taskinfo_s *ret = NULL;
	irqstate_t flags = px4_enter_critical_section();

	if (hashtab) {
		ret = hashtab[HASH(pid)];
	}

	px4_leave_critical_section(flags);
	return ret;
}

static void drop_task_info(pid_t pid)
{
	irqstate_t flags = px4_enter_critical_section();
	hashtab[HASH(pid)] = NULL;
	px4_leave_critical_section(flags);
}

static int hash_task_info(struct system_load_taskinfo_s *task_info, pid_t pid)
{
	struct system_load_taskinfo_s **newtab;
	void *temp;
	int hash;
	int i;

	/* Use critical section to protect the hash table */

	irqstate_t flags = px4_enter_critical_section();

	/* Keep trying until we get it or run out of memory */

retry:

	/* Calculate hash */

	hash = HASH(pid);

	/* Check if the entry is available or has this pid already (task restart case) */

	if (hashtab[hash] == NULL || task_info->tcb->pid == hashtab[hash]->tcb->pid) {
		hashtab[hash] = task_info;
		px4_leave_critical_section(flags);
		return OK;
	}

	/* No can do, double the size of the hash table */

	newtab = (struct system_load_taskinfo_s **)kmm_zalloc(hashtab_size * 2 * sizeof(*newtab));

	if (newtab == NULL) {
		px4_leave_critical_section(flags);
		return -ENOMEM;
	}

	hashtab_size *= 2;

	/* Start using the new hash table */

	for (i = 0; i < hashtab_size / 2; i++) {
		struct system_load_taskinfo_s *info = hashtab[i];

		if (info && info->tcb) {
			hash = HASH(info->tcb->pid);
			newtab[hash] = hashtab[i];

		} else {
			newtab[i] = NULL;
		}
	}

	temp = hashtab;
	hashtab = newtab;
	kmm_free(temp);

	/* Try again */

	goto retry;
}

static px4::atomic_int cpuload_monitor_all_count{0};

void cpuload_monitor_start()
{
	if (cpuload_monitor_all_count.fetch_add(1) == 0) {
		// if the count was previously 0 (idle thread only) then clear any existing runtime data
		irqstate_t flags = px4_enter_critical_section();

		system_load.start_time = hrt_absolute_time();

		for (int i = CONFIG_SMP_NCPUS; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
			system_load.tasks[i].total_runtime = 0;
			system_load.tasks[i].curr_start_time = 0;
		}

		px4_leave_critical_section(flags);
	}
}

void cpuload_monitor_stop()
{
	if (cpuload_monitor_all_count.fetch_sub(1) <= 1) {
		// don't allow the count to go negative
		cpuload_monitor_all_count.store(0);
	}
}

void cpuload_initialize_once()
{
	/* Initialize hashing */

	init_task_hash();

	for (auto &task : system_load.tasks) {
		task.valid = false;
	}

	// perform static initialization of "system" threads
	for (system_load.total_count = 0; system_load.total_count < CONFIG_SMP_NCPUS; system_load.total_count++) {
		system_load.tasks[system_load.total_count].total_runtime = 0;
		system_load.tasks[system_load.total_count].curr_start_time = 0;
		system_load.tasks[system_load.total_count].tcb = nxsched_get_tcb(
					system_load.total_count);	// it is assumed that these static threads have consecutive PIDs
		system_load.tasks[system_load.total_count].valid = true;
		hash_task_info(&system_load.tasks[system_load.total_count], system_load.total_count);
	}

	system_load.initialized = note_driver_register(&g_cpuload_driver) == 0;
}

void cpuload_start(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb)
{
	// find first free slot
	if (system_load.initialized) {
		for (auto &task : system_load.tasks) {
			if (!task.valid) {
				// slot is available
				task.total_runtime = 0;
				task.curr_start_time = 0;
				task.tcb = tcb;
				task.valid = true;
				system_load.total_count++;
				// add to the hashlist
				hash_task_info(&task, tcb->pid);
				break;
			}
		}
	}
}

void cpuload_stop(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
		for (auto &task : system_load.tasks) {
			if (task.tcb && task.tcb->pid == tcb->pid) {
				// mark slot as free
				task.valid = false;
				task.total_runtime = 0;
				task.curr_start_time = 0;
				task.tcb = nullptr;
				system_load.total_count--;
				// drop from the tasklist
				drop_task_info(tcb->pid);
				break;
			}
		}
	}
}

void cpuload_suspend(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
		if (tcb->pid < CONFIG_SMP_NCPUS) {
			system_load.tasks[tcb->pid].total_runtime += hrt_elapsed_time(&system_load.tasks[tcb->pid].curr_start_time);
			return;

		} else {
			if (cpuload_monitor_all_count.load() == 0) {
				return;
			}
		}

		struct system_load_taskinfo_s *task = get_task_info(tcb->pid);

		if (task) {
			// Task ending its current scheduling run
			if (task->valid && (task->curr_start_time > 0)
			    && task->tcb && task->tcb->pid == tcb->pid) {
				task->total_runtime += hrt_elapsed_time(&task->curr_start_time);
			}
		}
	}
}

void cpuload_resume(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
		if (tcb->pid < CONFIG_SMP_NCPUS) {
			hrt_store_absolute_time(&system_load.tasks[tcb->pid].curr_start_time);
			return;

		} else {
			if (cpuload_monitor_all_count.load() == 0) {
				return;
			}
		}

		struct system_load_taskinfo_s *task = get_task_info(tcb->pid);

		if (task) {
			if (task->valid && task->tcb && task->tcb->pid == tcb->pid) {
				// curr_start_time is accessed from an IRQ handler (in logger), so we need
				// to make the update atomic
				hrt_store_absolute_time(&task->curr_start_time);
			}
		}
	}
}

__END_DECLS
#endif // PX4_NUTTX && CONFIG_SCHED_INSTRUMENTATION
