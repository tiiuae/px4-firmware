/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file perf_counter.cpp
 *
 * @brief Performance measuring tools.
 */

#ifndef MODULE_NAME
#define MODULE_NAME "perf"
#endif

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <systemlib/err.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <errno.h>
#include <limits.h>
#include <new>
#include <unistd.h>
#include <px4_platform_common/time.h>

#include <px4_platform_common/mmap.h>

#include "perf_counter.h"

#include <px4_platform_common/atomic.h>

// perf counters are written on the hot path (perf_begin/perf_end/perf_count)
// and read concurrently by the 'perf' command and the logger.
//
// On POSIX/SITL we always use real atomics: ThreadSanitizer needs them to stay
// clean, native 64-bit atomics are cheap there, and we don't care about the
// overhead. On flight hardware (NuttX/QURT) these are single-writer diagnostic
// counters, and a 64-bit atomic lowers to a locked (interrupt-disabling)
// sequence on Cortex-M (~2x cost on this hot path). There a concurrent reader
// can at worst observe an off-by-one torn read of a 64-bit field, which is
// benign and self-correcting, so a plain scalar is used instead.
//
// PerfVar mirrors the subset of the px4::atomic API used below so the call
// sites are identical in both configurations; on embedded the accessors are
// trivial and inline to bare loads/stores.
#if defined(__PX4_POSIX) || defined(__SANITIZE_THREAD__)
template <typename T> using PerfVar = px4::atomic<T>;
#else
template <typename T>
class PerfVar
{
public:
	PerfVar() = default;
	T load() const { return _value; }
	void store(T value) { _value = value; }
	T fetch_add(T value) { const T previous = _value; _value += value; return previous; }
private:
	T _value {};
};
#endif

#define PERF_SHMNAME_PREFIX	"_perf_"
#define PERF_SHMNAME_STR	PERF_SHMNAME_PREFIX "%s"
#define PERF_SHMNAME_MAX	NAME_MAX + 1

#ifndef CONFIG_FS_SHMFS_VFS_PATH
#define CONFIG_FS_SHMFS_VFS_PATH "/dev/shm"
#endif

/**
 * Header common to all counters.
 */
struct perf_ctr_header {
	enum perf_counter_type	type;	/**< counter type */
	px4::atomic<bool>	ready{false};
	px4::atomic<bool>	unlinked{false};
	char 			name[PERF_SHMNAME_MAX]; /**< counter name */
	char			shmname[PERF_SHMNAME_MAX];
};

/**
 * PC_EVENT counter.
 */
struct perf_ctr_count : public perf_ctr_header {
	PerfVar<uint64_t>	event_count;
};

/**
 * PC_ELAPSED counter.
 */
struct perf_ctr_elapsed : public perf_ctr_header {
	PerfVar<uint64_t>	event_count;
	PerfVar<uint64_t>	time_start;
	PerfVar<uint64_t>	time_total;
	PerfVar<uint32_t>	time_least;
	PerfVar<uint32_t>	time_most;
	PerfVar<float>	mean;
	PerfVar<float>	M2;
};

/**
 * PC_INTERVAL counter.
 */
struct perf_ctr_interval : public perf_ctr_header {
	PerfVar<uint64_t>	event_count;
	PerfVar<uint64_t>	time_event;
	PerfVar<uint64_t>	time_first;
	PerfVar<uint64_t>	time_last;
	PerfVar<uint32_t>	time_least;
	PerfVar<uint32_t>	time_most;
	PerfVar<float>	mean;
	PerfVar<float>	M2;
};

static size_t perf_size(enum perf_counter_type type)
{
	switch (type) {
	case PC_COUNT:
		return sizeof(perf_ctr_count);

	case PC_ELAPSED:
		return sizeof(perf_ctr_elapsed);

	case PC_INTERVAL:
		return sizeof(perf_ctr_interval);

	default:
		break;
	}

	return 0;
}

static size_t perf_shmsize(int fd)
{
	struct stat sb {};

	if (fstat(fd, &sb) < 0) {
		PX4_ERR("Cannot stat %d", fd);
		return 0;
	}

	return sb.st_size;
}

static int perf_shmname(char *shmname, const char *name, size_t size, bool once)
{
	static px4::atomic<unsigned> sequence{0};
	const int ret = once ? snprintf(shmname, size, PERF_SHMNAME_STR, name) :
			snprintf(shmname, size, PERF_SHMNAME_PREFIX "%ld_%u_%s",
				 (long)getpid(), sequence.fetch_add(1), name);

	if (ret < 0 || (size_t)ret >= size) {
		return -ENAMETOOLONG;
	}

	return OK;
}

using perf_handle_callback = void (*)(perf_counter_t, void *);

static void perf_foreach(perf_handle_callback cb, void *arg = nullptr)
{
	DIR *dir = opendir(CONFIG_FS_SHMFS_VFS_PATH);

	if (dir == nullptr) {
		PX4_ERR("Cannot open perf directory: %d", errno);
		return;
	}

	struct dirent *perf;

	while ((perf = readdir(dir)) != nullptr) {
		if (!strncmp(perf->d_name, PERF_SHMNAME_PREFIX, sizeof(PERF_SHMNAME_PREFIX) - 1)) {
			/* found the counter, map it for us */
			int fd = shm_open(perf->d_name, O_RDWR, 0666);

			if (fd >= 0) {
				size_t size = perf_shmsize(fd);

				if (size < sizeof(perf_ctr_header)) {
					close(fd);
					continue;
				}

				void *p = px4_mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

				/* the file can be closed now */
				close(fd);

				if (p != MAP_FAILED) {
					perf_counter_t handle = (perf_counter_t)p;

					if (handle->ready.load() && perf_size(handle->type) == size) {
						cb(handle, arg);
					}

					px4_munmap(p, size);

				} else {
					PX4_ERR("Cannot map perf counter %s: %d", perf->d_name, errno);
				}

			} else if (errno != ENOENT) {
				PX4_ERR("Cannot open perf counter %s: %d", perf->d_name, errno);
			}
		}
	}

	closedir(dir);
}

static void perf_print_cb(perf_counter_t handle, void *arg)
{
	perf_print_counter(handle);
}

static void perf_reset_cb(perf_counter_t handle, void *arg)
{
	perf_reset(handle);
}

static perf_counter_t perf_allocate(enum perf_counter_type type, const char *name, bool once)
{
	const size_t size = perf_size(type);

	if (name == nullptr || size == 0) {
		PX4_ERR("Invalid perf counter type or name");
		return nullptr;
	}

	char shmname[PERF_SHMNAME_MAX];
	int ret = perf_shmname(shmname, name, sizeof(shmname), true);

	if (ret < 0) {
		PX4_ERR("failed to allocate perf counter %s", name);
		return nullptr;
	}

	int fd = shm_open(shmname, O_CREAT | O_RDWR | O_EXCL, 0666);

	while (fd < 0 && !once && errno == EEXIST) {
		if (perf_shmname(shmname, name, sizeof(shmname), false) < 0) {
			PX4_ERR("Perf counter name too long: %s", name);
			return nullptr;
		}

		fd = shm_open(shmname, O_CREAT | O_RDWR | O_EXCL, 0666);
	}

	const bool created = fd >= 0;

	if (!created && once && errno == EEXIST) {
		fd = shm_open(shmname, O_RDWR, 0666);
	}

	if (fd < 0) {
		PX4_ERR("Cannot allocate perf counter %s: %d", name, errno);
		return nullptr;
	}

	if (created) {
		if (ftruncate(fd, size) != 0) {
			PX4_ERR("Cannot size perf counter %s: %d", name, errno);
			close(fd);
			shm_unlink(shmname);
			return nullptr;
		}

	} else {
		for (unsigned retry = 0; retry < 100 && perf_shmsize(fd) == 0; ++retry) {
			px4_usleep(1000);
		}

		if (perf_shmsize(fd) != size) {
			PX4_ERR("Perf counter size mismatch: %s", name);
			close(fd);
			return nullptr;
		}
	}

	void *p = px4_mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);

	if (p == MAP_FAILED) {
		PX4_ERR("Cannot map perf counter %s: %d", name, errno);

		if (created) {
			shm_unlink(shmname);
		}

		return nullptr;
	}

	perf_counter_t ctr = (perf_counter_t)p;

	if (created) {
		switch (type) {
		case PC_COUNT: ctr = new (p) perf_ctr_count(); break;

		case PC_ELAPSED: ctr = new (p) perf_ctr_elapsed(); break;

		case PC_INTERVAL: ctr = new (p) perf_ctr_interval(); break;
		}

		ctr->type = type;
		strcpy(ctr->name, name);
		strcpy(ctr->shmname, shmname);
		ctr->ready.store(true);

	} else {
		for (unsigned retry = 0; retry < 100 && !ctr->ready.load(); ++retry) {
			px4_usleep(1000);
		}

		if (!ctr->ready.load() || ctr->type != type || strcmp(ctr->name, name) != 0) {
			PX4_ERR("Perf counter not ready or type mismatch: %s", name);
			px4_munmap(ctr, size);
			return nullptr;
		}
	}

	return ctr;
}

perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{
	return perf_allocate(type, name, false);
}

perf_counter_t
perf_alloc_once(enum perf_counter_type type, const char *name)
{
	return perf_allocate(type, name, true);
}

void
perf_free(perf_counter_t handle)
{
	if (handle) {
		bool unlinked = false;

		if (handle->unlinked.compare_exchange(&unlinked, true) && shm_unlink(handle->shmname) < 0 && errno != ENOENT) {
			PX4_ERR("Cannot unlink perf counter %s: %d", handle->name, errno);
		}

		px4_munmap(handle, perf_size(handle->type));
	}
}

void
perf_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count.fetch_add(1);
		break;

	case PC_INTERVAL:
		perf_count_interval(handle, hrt_absolute_time());
		break;

	default:
		break;
	}
}

void
perf_begin(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((struct perf_ctr_elapsed *)handle)->time_start.store(hrt_absolute_time());
		break;

	default:
		break;
	}
}

void
perf_end(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			hrt_abstime ts = pce->time_start.load();

			if (ts != 0) {
				perf_set_elapsed(handle, hrt_absolute_time() - ts);
			}
		}
		break;

	default:
		break;
	}
}

void
perf_set_elapsed(perf_counter_t handle, int64_t elapsed)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (elapsed >= 0) {
				uint64_t ec = pce->event_count.fetch_add(1) + 1;
				pce->time_total.fetch_add((uint64_t)elapsed);

				uint32_t tl = pce->time_least.load();

				if ((tl > (uint32_t)elapsed) || (tl == 0)) {
					pce->time_least.store((uint32_t)elapsed);
				}

				uint32_t tm = pce->time_most.load();

				if (tm < (uint32_t)elapsed) {
					pce->time_most.store((uint32_t)elapsed);
				}

				// maintain mean and variance of the elapsed time in seconds
				// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
				float dt = elapsed / 1e6f;
				float mean_val = pce->mean.load();
				float delta_intvl = dt - mean_val;
				mean_val += delta_intvl / ec;
				pce->mean.store(mean_val);
				float m2_val = pce->M2.load();
				m2_val += delta_intvl * (dt - mean_val);
				pce->M2.store(m2_val);

				pce->time_start.store((uint64_t)0);
			}
		}
		break;

	default:
		break;
	}
}

void
perf_count_interval(perf_counter_t handle, hrt_abstime now)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			uint64_t ec = pci->event_count.load();

			switch (ec) {
			case 0:
				pci->time_first.store(now);
				break;

			case 1: {
					uint64_t tl = pci->time_last.load();
					uint32_t interval = (uint32_t)(now - tl);
					pci->time_least.store(interval);
					pci->time_most.store(interval);
					pci->mean.store(interval / 1e6f);
					pci->M2.store(0.0f);
					break;
				}

			default: {
					uint64_t tl = pci->time_last.load();
					hrt_abstime interval = now - tl;

					uint32_t least = pci->time_least.load();

					if ((uint32_t)interval < least) {
						pci->time_least.store((uint32_t)interval);
					}

					uint32_t most = pci->time_most.load();

					if ((uint32_t)interval > most) {
						pci->time_most.store((uint32_t)interval);
					}

					// maintain mean and variance of interval in seconds
					// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
					float dt = interval / 1e6f;
					float mean_val = pci->mean.load();
					float delta_intvl = dt - mean_val;
					mean_val += delta_intvl / ec;
					pci->mean.store(mean_val);
					float m2_val = pci->M2.load();
					m2_val += delta_intvl * (dt - mean_val);
					pci->M2.store(m2_val);
					break;
				}
			}

			pci->time_last.store(now);
			pci->event_count.fetch_add(1);
			break;
		}

	default:
		break;
	}
}

void
perf_set_count(perf_counter_t handle, uint64_t count)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT: {
			((struct perf_ctr_count *)handle)->event_count.store(count);
		}
		break;

	default:
		break;
	}

}

void
perf_cancel(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			pce->time_start.store((uint64_t)0);
		}
		break;

	default:
		break;
	}
}

void
perf_reset(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count.store((uint64_t)0);
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			pce->event_count.store((uint64_t)0);
			pce->time_start.store((uint64_t)0);
			pce->time_total.store((uint64_t)0);
			pce->time_least.store((uint32_t)0);
			pce->time_most.store((uint32_t)0);
			pce->mean.store(0.0f);
			pce->M2.store(0.0f);
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			pci->event_count.store((uint64_t)0);
			pci->time_event.store((uint64_t)0);
			pci->time_first.store((uint64_t)0);
			pci->time_last.store((uint64_t)0);
			pci->time_least.store((uint32_t)0);
			pci->time_most.store((uint32_t)0);
			pci->mean.store(0.0f);
			pci->M2.store(0.0f);
			break;
		}
	}
}

void
perf_print_counter(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT: {
			uint64_t ec = ((struct perf_ctr_count *)handle)->event_count.load();
			PX4_INFO_RAW("%s: %" PRIu64 " events\n",
				     handle->name, ec);
			break;
		}

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			uint64_t ec = pce->event_count.load();
			uint64_t tt = pce->time_total.load();
			uint32_t tl = pce->time_least.load();
			uint32_t tm = pce->time_most.load();
			float m2 = pce->M2.load();
			float rms = sqrtf(m2 / (ec - 1));
			PX4_INFO_RAW("%s: %" PRIu64 " events, %" PRIu64 "us elapsed, %.2fus avg, min %" PRIu32 "us max %" PRIu32
				     "us %5.3fus rms\n",
				     handle->name, ec, tt,
				     (ec == 0) ? 0 : (double)tt / (double)ec,
				     tl, tm, (double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			uint64_t ec = pci->event_count.load();
			uint64_t tf = pci->time_first.load();
			uint64_t tla = pci->time_last.load();
			uint32_t tl = pci->time_least.load();
			uint32_t tm = pci->time_most.load();
			float m2 = pci->M2.load();
			float rms = sqrtf(m2 / (ec - 1));

			PX4_INFO_RAW("%s: %" PRIu64 " events, %.2fus avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms\n",
				     handle->name, ec,
				     (ec == 0) ? 0 : (double)(tla - tf) / (double)ec,
				     tl, tm, (double)(1e6f * rms));
			break;
		}

	default:
		break;
	}
}


int
perf_print_counter_buffer(char *buffer, int length, perf_counter_t handle)
{
	int num_written = 0;

	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		num_written = snprintf(buffer, length, "%s: %" PRIu64 " events",
				       handle->name,
				       ((struct perf_ctr_count *)handle)->event_count.load());
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			uint64_t ec = pce->event_count.load();
			uint64_t tt = pce->time_total.load();
			uint32_t tl = pce->time_least.load();
			uint32_t tm = pce->time_most.load();
			float m2 = pce->M2.load();
			float rms = sqrtf(m2 / (ec - 1));
			num_written = snprintf(buffer, length,
					       "%s: %" PRIu64 " events, %" PRIu64 "us elapsed, %.2fus avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms",
					       handle->name,
					       ec,
					       tt,
					       (ec == 0) ? 0 : (double)tt / (double)ec,
					       tl,
					       tm,
					       (double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			uint64_t ec = pci->event_count.load();
			uint32_t tl = pci->time_least.load();
			uint32_t tm = pci->time_most.load();
			float m2 = pci->M2.load();
			float rms = sqrtf(m2 / (ec - 1));

			num_written = snprintf(buffer, length,
					       "%s: %" PRIu64 " events, %.2f avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms",
					       handle->name,
					       ec,
					       (ec == 0) ? 0 : (double)(pci->time_last.load() - pci->time_first.load()) / (double)ec,
					       tl,
					       tm,
					       (double)(1e6f * rms));
			break;
		}

	default:
		break;
	}

	buffer[length - 1] = 0; // ensure 0-termination
	return num_written;
}

uint64_t
perf_event_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		return ((struct perf_ctr_count *)handle)->event_count.load();

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			return pce->event_count.load();
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			return pci->event_count.load();
		}

	default:
		break;
	}

	return 0;
}

float
perf_mean(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			return pce->mean.load();
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			return pci->mean.load();
		}

	default:
		break;
	}

	return 0.0f;
}

struct perf_iteration_context {
	perf_callback callback;
	void *user;
};

static void perf_iterate_cb(perf_counter_t handle, void *arg)
{
	auto *context = static_cast<perf_iteration_context *>(arg);
	static constexpr size_t PERF_COUNTER_LINE_BUFFER_SIZE = 220;
	char buffer[PERF_COUNTER_LINE_BUFFER_SIZE];

	perf_print_counter_buffer(buffer, sizeof(buffer), handle);
	context->callback(buffer, context->user);
}

void
perf_iterate_all(perf_callback cb, void *user)
{
	perf_iteration_context context{cb, user};
	perf_foreach(perf_iterate_cb, &context);
}

void
perf_print_all(void)
{
	perf_foreach(perf_print_cb);
}

void
perf_print_latency(void)
{
	latency_info_t latency;
	PX4_INFO_RAW("bucket [us] : events\n");

	for (int i = 0; i < get_latency_bucket_count(); i++) {
		latency = get_latency(i, i);
		PX4_INFO_RAW("       %4i : %li\n", latency.bucket, (long int)latency.counter);
	}

	// print the overflow bucket value
	latency = get_latency(get_latency_bucket_count() - 1, get_latency_bucket_count());
	PX4_INFO_RAW(" >%4" PRIu16 " : %" PRIu32 "\n", latency.bucket, latency.counter);
}

void
perf_reset_all(void)
{
	perf_foreach(perf_reset_cb);
	reset_latency_counters();
}
