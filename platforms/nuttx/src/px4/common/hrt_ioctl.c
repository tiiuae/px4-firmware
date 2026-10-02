/****************************************************************************
 *
 *   Copyright (c) 2022 Technology Innovation Institute. All rights reserved.
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

#include <stdbool.h>

#include <px4_platform_common/px4_config.h>

#include <px4_platform/board_ctrl.h>
#include <px4_platform/micro_hal.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/sem.h>

#include <drivers/drv_hrt.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/spinlock.h>
#include <queue.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#ifndef MODULE_NAME
#  define MODULE_NAME "hrt_ioctl"
#endif

struct usr_hrt_call {
	struct sq_entry_s list_item; /* List head for local sl_list */
	struct hrt_call entry;       /* Kernel side entry for HRT driver */
	struct hrt_call *usr_entry;  /* Reference to user side entry */
};

static sq_queue_t callout_queue;
static sq_queue_t callout_freelist;
static sq_queue_t callout_inflight;

/* SMP spinlock for g_hrt_ioctl_lock.
 *
 * Note: when SMP=no the spin lock turns into normal critical section i.e. it
 * only disables interrupts
 */

static spinlock_t g_hrt_ioctl_lock = SP_UNLOCKED;

/* Check if entry is in list */

static bool entry_inlist(sq_queue_t *queue, sq_entry_t *item)
{
	sq_entry_t *queued;

	sq_for_every(queue, queued) {
		if (queued == item) {
			return true;
		}
	}

	return false;
}

/* Find (pop) first entry for user from queue, the queue must be locked prior */

static struct usr_hrt_call *pop_user(sq_queue_t *queue, const px4_hrt_handle_t handle)
{
	sq_entry_t *queued;

	sq_for_every(queue, queued) {
		struct usr_hrt_call *e = (void *)queued;

		if (e->entry.callout_sem == handle) {
			sq_rem(queued, queue);
			return e;
		}
	}

	return NULL;
}

/* Find (pop) entry from queue, the queue must be locked prior */

static struct usr_hrt_call *pop_entry(sq_queue_t *queue, const px4_hrt_handle_t handle, struct hrt_call *entry)
{
	sq_entry_t *queued;

	sq_for_every(queue, queued) {
		struct usr_hrt_call *e = (void *)queued;

		if (e->usr_entry == entry && e->entry.callout_sem == handle) {
			sq_rem(queued, queue);
			return e;
		}
	}

	return NULL;
}

/**
 * Copy user entry to kernel space. Either re-uses existing one or if none can
 * be found, creates a new.
 *
 * handle  : user space handle to identify who is behind the HRT request
 * entry   : user space HRT entry
 * callout : user callback
 * arg     : user argument passed in callback
 */
static struct usr_hrt_call *dup_entry(const px4_hrt_handle_t handle, struct hrt_call *entry, hrt_callout callout,
				      void *arg)
{
	struct usr_hrt_call *e = NULL;

	irqstate_t flags = spin_lock_irqsave_notrace(&g_hrt_ioctl_lock);

	/* check if this is already queued */
	e = pop_entry(&callout_queue, handle, entry);

	/* it was not already queued, get from freelist */
	if (!e) {
		e = (void *)sq_remfirst(&callout_freelist);
	}

	spin_unlock_irqrestore_notrace(&g_hrt_ioctl_lock, flags);

	if (!e) {
		/* Allocate a new kernel side item for the user call */

		e = kmm_malloc(sizeof(struct usr_hrt_call));
	}

	if (e) {

		/* Store the user side callout function and argument to the user's handle */
		entry->callout = callout;
		entry->arg = arg;

		/* Store reference to the kernel side entry to the user side struct and
		 * references to the semaphore and user side entry to the kernel side item
		 */

		e->entry.callout_sem = handle;
		e->usr_entry = entry;

		/* Add this to the callout_queue list */
		flags = spin_lock_irqsave_notrace(&g_hrt_ioctl_lock);
		sq_addfirst(&e->list_item, &callout_queue);
		spin_unlock_irqrestore_notrace(&g_hrt_ioctl_lock, flags);

	} else {
		PX4_ERR("out of memory");

	}

	return e;
}

void hrt_usr_call(void *arg)
{
	// This is called from hrt interrupt
	struct usr_hrt_call *e = (struct usr_hrt_call *)arg;
	bool post = false;
	irqstate_t flags = spin_lock_irqsave_notrace(&g_hrt_ioctl_lock);

	// Make sure the event is not already in flight
	if (!entry_inlist(&callout_inflight, (sq_entry_t *)e)) {
		sq_rem(&e->list_item, &callout_queue);
		sq_addfirst(&e->list_item, &callout_inflight);
		post = true;
	}

	spin_unlock_irqrestore_notrace(&g_hrt_ioctl_lock, flags);

	if (post) {
		flags = enter_critical_section();

		if (e->entry.callout_sem) {
			px4_sem_post(e->entry.callout_sem);
		}

		leave_critical_section(flags);
	}
}

int hrt_ioctl(unsigned int cmd, unsigned long arg);

void hrt_ioctl_init(void)
{
	sq_init(&callout_queue);
	sq_init(&callout_freelist);
	sq_init(&callout_inflight);

	/* register ioctl callbacks */
	px4_register_boardct_ioctl(_HRTIOCBASE, hrt_ioctl);
}

/* These functions are inlined in all but NuttX protected/kernel builds */

latency_info_t get_latency(uint16_t bucket_idx, uint16_t counter_idx)
{
	latency_info_t ret = {latency_buckets[bucket_idx], latency_counters[counter_idx]};
	return ret;
}

void reset_latency_counters(void)
{
	for (int i = 0; i <= get_latency_bucket_count(); i++) {
		latency_counters[i] = 0;
	}
}

/* board_ioctl interface for user-space hrt driver */

#define HRT_CLIENTS 64

static struct {
	px4_sem_t *sem;
	pid_t owner;
} g_hrt_clients[HRT_CLIENTS];

static px4_sem_t g_hrt_clients_lock = SEM_INITIALIZER(1);

static px4_sem_t *hrt_client(px4_hrt_handle_t handle)
{
	uintptr_t i = (uintptr_t)handle - 1;

	return i < HRT_CLIENTS && g_hrt_clients[i].owner == getpid() ? g_hrt_clients[i].sem : NULL;
}

static void hrt_unregister(px4_sem_t *callback_sem)
{
	sq_entry_t *queued;
	sq_queue_t deleted;
	struct usr_hrt_call *e;
	irqstate_t flags;

	sq_init(&deleted);

	flags = spin_lock_irqsave_notrace(&g_hrt_ioctl_lock);

	sq_for_every(&callout_queue, queued) {
		e = container_of(queued, struct usr_hrt_call, list_item);

		if (callback_sem == e->entry.callout_sem) {
			sq_rem(&e->list_item, &callout_queue);
			hrt_cancel(&e->entry);

			/* Remove potential inflight entry as well */
			sq_rem(&e->list_item, &callout_inflight);

			/* Add this to a local deleted list */
			sq_addfirst(&e->list_item, &deleted);
		}
	}

	spin_unlock_irqrestore_notrace(&g_hrt_ioctl_lock, flags);

	/* Perhaps the HRT alrady fired before entering the spinlock above, and
	 * the interrupt handler is running on the other CPU.
	 * Set callout_sem to NULL for each deleted entry before destroying the
	 * semaphore
	 */

	flags = enter_critical_section();
	sq_for_every(&deleted, queued) {
		e = container_of(queued, struct usr_hrt_call, list_item);
		e->entry.callout_sem = NULL;
	}

	px4_sem_destroy(callback_sem);
	leave_critical_section(flags);

	/* Free all the memory */

	sq_for_every(&deleted, queued) {
		e = container_of(queued, struct usr_hrt_call, list_item);
		kmm_free(e);
	}

	kmm_free(callback_sem);
}

static int hrt_register(px4_hrt_handle_t *handle)
{
	int ret = -ENOMEM;

	px4_sem_wait(&g_hrt_clients_lock);

	for (int i = 0; i < HRT_CLIENTS; i++) {
		if (g_hrt_clients[i].sem != NULL && kill(g_hrt_clients[i].owner, 0) < 0) {
			hrt_unregister(g_hrt_clients[i].sem);
			g_hrt_clients[i].sem = NULL;
		}

		if (g_hrt_clients[i].sem == NULL) {
			px4_sem_t *callback_sem = kmm_malloc(sizeof(px4_sem_t));

			/* Create a semaphore for handling hrt driver callbacks */
			if (callback_sem != NULL && px4_sem_init(callback_sem, 0, 0) == 0) {

				/* this is a signalling semaphore */
				px4_sem_setprotocol(callback_sem, SEM_PRIO_NONE);
				g_hrt_clients[i].sem = callback_sem;
				g_hrt_clients[i].owner = getpid();
				*handle = (px4_hrt_handle_t)(uintptr_t)(i + 1);
				ret = OK;

			} else {
				kmm_free(callback_sem);
			}

			break;
		}
	}

	px4_sem_post(&g_hrt_clients_lock);

	if (ret != OK) {
		*handle = NULL;
	}

	return ret;
}

int
hrt_ioctl(unsigned int cmd, unsigned long arg)
{
	hrt_boardctl_t h;
	px4_sem_t *callout_sem;

	switch (cmd) {
	case HRT_WAITEVENT: {
			irqstate_t flags;
			struct usr_hrt_call *e;

			if (!px4_user_ok((void *)arg, sizeof(h)) || (callout_sem = hrt_client(((hrt_boardctl_t *)arg)->handle)) == NULL) {
				return -EFAULT;
			}

			do { } while (px4_sem_wait(callout_sem) != 0);

			/* Atomically update the pointer to user side hrt entry */
			flags = spin_lock_irqsave_notrace(&g_hrt_ioctl_lock);
			e = pop_user(&callout_inflight, callout_sem);

			if (e) {
				((hrt_boardctl_t *)arg)->callout = e->usr_entry->callout;
				((hrt_boardctl_t *)arg)->arg = e->usr_entry->arg;

				// If the period is 0, the callout is no longer queued by hrt driver
				// move it back to freelist
				if (e->entry.period == 0) {
					sq_addfirst((sq_entry_t *)e, &callout_freelist);

				} else {
					sq_addfirst((sq_entry_t *)e, &callout_queue);
				}

			} else {
				PX4_ERR("HRT_WAITEVENT error no entry");
			}

			spin_unlock_irqrestore_notrace(&g_hrt_ioctl_lock, flags);
		}
		break;

	case HRT_ABSOLUTE_TIME:
		if (arg == 0 || !px4_user_ok((void *)arg, sizeof(hrt_abstime))) {
			return -EFAULT;
		}

		*(hrt_abstime *)arg = hrt_absolute_time();
		break;

	case HRT_CALL_AFTER:
	case HRT_CALL_AT:
	case HRT_CALL_EVERY: {
			struct usr_hrt_call *e;

			if (!px4_user_ok((void *)arg, sizeof(h)) || arg == 0) {
				return -EFAULT;
			}

			memcpy(&h, (void *)arg, sizeof(h));

			if ((callout_sem = hrt_client(h.handle)) == NULL || h.entry == NULL
			    || !px4_user_ok(h.entry, sizeof(*h.entry))) {
				return -EFAULT;
			}

			e = dup_entry(callout_sem, h.entry, h.callout, h.arg);

			if (e && cmd == HRT_CALL_AFTER) {
				hrt_call_after(&e->entry, h.time, (hrt_callout)hrt_usr_call, e);

			} else if (e && cmd == HRT_CALL_AT) {
				hrt_call_at(&e->entry, h.time, (hrt_callout)hrt_usr_call, e);

			} else if (e) {
				hrt_call_every(&e->entry, h.time, h.interval, (hrt_callout)hrt_usr_call, e);
			}
		}
		break;

	case HRT_CANCEL: {
			irqstate_t flags;
			struct usr_hrt_call *e;

			if (!px4_user_ok((void *)arg, sizeof(h)) || arg == 0) {
				return -EFAULT;
			}

			memcpy(&h, (void *)arg, sizeof(h));

			if ((callout_sem = hrt_client(h.handle)) == NULL || h.entry == NULL) {
				return -EFAULT;
			}

			/* Find the user entry */
			flags = spin_lock_irqsave_notrace(&g_hrt_ioctl_lock);
			e = pop_entry(&callout_queue, callout_sem, h.entry);
			spin_unlock_irqrestore_notrace(&g_hrt_ioctl_lock, flags);

			if (e) {
				hrt_cancel(&e->entry);

			} else {
				/* If the HRT already triggered, it is in inflight queue */

				flags = spin_lock_irqsave_notrace(&g_hrt_ioctl_lock);
				e = pop_entry(&callout_inflight, callout_sem, h.entry);
				spin_unlock_irqrestore_notrace(&g_hrt_ioctl_lock, flags);
			}

			if (e) {
				flags = spin_lock_irqsave_notrace(&g_hrt_ioctl_lock);
				sq_addfirst((sq_entry_t *)e, &callout_freelist);
				spin_unlock_irqrestore_notrace(&g_hrt_ioctl_lock, flags);

			} else {
				PX4_ERR("HRT_CANCEL called with invalid entry\n");
			}
		}
		break;

	case HRT_GET_LATENCY: {
			latency_boardctl_t *latency = (latency_boardctl_t *)arg;

			if (arg == 0 || !px4_user_ok(latency, sizeof(*latency))) {
				return -EFAULT;
			}

			const uint16_t bucket_idx = latency->bucket_idx;
			const uint16_t counter_idx = latency->counter_idx;

			if (bucket_idx >= LATENCY_BUCKET_COUNT || counter_idx > LATENCY_BUCKET_COUNT) {
				return -EINVAL;
			}

			latency->latency = get_latency(bucket_idx, counter_idx);
		}
		break;

	case HRT_RESET_LATENCY:
		reset_latency_counters();
		break;

	case HRT_REGISTER:
		if (arg == 0 || !px4_user_ok((void *)arg, sizeof(px4_hrt_handle_t))) {
			return -EFAULT;
		}

		return hrt_register((px4_hrt_handle_t *)arg);

	case HRT_UNREGISTER: {
			if (arg == 0 || !px4_user_ok((void *)arg, sizeof(px4_hrt_handle_t))) {
				return -EFAULT;
			}

			const uintptr_t i = (uintptr_t)(*(px4_hrt_handle_t *)arg) - 1;

			px4_sem_wait(&g_hrt_clients_lock);
			callout_sem = hrt_client((px4_hrt_handle_t)(i + 1));

			if (callout_sem != NULL) {
				hrt_unregister(callout_sem);
				g_hrt_clients[i].sem = NULL;
			}

			px4_sem_post(&g_hrt_clients_lock);
			*(px4_hrt_handle_t *)arg = NULL;

			if (callout_sem == NULL) {
				return -EFAULT;
			}
		}
		break;

	case HRT_ABSTIME_BASE:
		if (arg == 0 || !px4_user_ok((void *)arg, sizeof(uintptr_t))) {
			return -EFAULT;
		}

#ifdef PX4_USERSPACE_HRT
		*(uintptr_t *)arg = hrt_absolute_time_usr_base();
#else
		*(uintptr_t *)arg = (uintptr_t)NULL;
#endif
		break;

	default:
		return -EINVAL;
	}

	return OK;
}
