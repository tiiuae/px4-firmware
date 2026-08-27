/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file log_utils.cpp
 */

#include "log_utils.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#ifdef __PX4_NUTTX
#define LOG_UTILS_DIRENT_DIR  DTYPE_DIRECTORY
#define LOG_UTILS_DIRENT_FILE DTYPE_FILE
#define LOG_UTILS_DIRENT_LINK DTYPE_LINK
#else
#define LOG_UTILS_DIRENT_DIR  DT_DIR
#define LOG_UTILS_DIRENT_FILE DT_REG
#define LOG_UTILS_DIRENT_LINK DT_LNK
#endif

namespace log_utils
{

static const char *const kLogRoot = PX4_STORAGEDIR "/log";

const char *log_root()
{
	return kLogRoot;
}

/* Running totals for one walk. Kept in a struct so the recursion carries a
 * single pointer rather than several by-reference parameters.
 */
struct WalkResult {
	bool erase;        ///< true: remove entries, false: only count them
	int  entries;      ///< files counted, or entries removed
	int  failures;     ///< entries that could not be removed
};

/**
 * Walk the directory whose path is in @p path, which must be a NUL-terminated
 * buffer of @p cap bytes with room to append "/<name>".
 */
static void walk(char *path, size_t cap, unsigned depth_left, WalkResult &result)
{
	DIR *dp = opendir(path);

	if (dp == nullptr) {
		PX4_WARN("cannot open %s: %d", path, errno);
		result.failures++;
		return;
	}

	const size_t base_len = strlen(path);
	struct dirent *entry = nullptr;

	while ((entry = readdir(dp)) != nullptr) {
		if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
			continue;
		}

		const int written = snprintf(path + base_len, cap - base_len, "/%s", entry->d_name);

		if (written <= 0 || (size_t)written >= cap - base_len) {
			// The original walk skipped over-long paths silently; say so instead.
			PX4_WARN("path too long, skipping %s/%s", path, entry->d_name);
			path[base_len] = '\0';
			result.failures++;
			continue;
		}

		switch (entry->d_type) {
		case LOG_UTILS_DIRENT_DIR:
			if (depth_left == 0) {
				PX4_WARN("depth limit reached, not descending into %s", path);
				result.failures++;
				break;
			}

			walk(path, cap, depth_left - 1, result);

			if (result.erase && rmdir(path) != 0) {
				PX4_WARN("cannot remove directory %s: %d", path, errno);
				result.failures++;

			} else if (result.erase) {
				result.entries++;
			}

			break;

		case LOG_UTILS_DIRENT_LINK:

		case LOG_UTILS_DIRENT_FILE:
			if (result.erase) {
				if (unlink(path) != 0) {
					PX4_WARN("cannot delete %s: %d", path, errno);
					result.failures++;

				} else {
					result.entries++;
				}

			} else {
				result.entries++;
			}

			break;

		default:
			PX4_WARN("skipping %s: unexpected type %d", path, (int)entry->d_type);
			result.failures++;
			break;
		}

		path[base_len] = '\0';
	}

	closedir(dp);
}

/* Seed a walk: copy the root into the shared buffer and run. */
static int run_walk(bool erase, unsigned max_depth, int &entries)
{
	char path[PATH_MAX];

	if (strlen(kLogRoot) >= sizeof(path)) {
		return -ENAMETOOLONG;
	}

	strncpy(path, kLogRoot, sizeof(path) - 1);
	path[sizeof(path) - 1] = '\0';

	// Fail cleanly when there is no card rather than reporting an empty tree.
	DIR *probe = opendir(path);

	if (probe == nullptr) {
		return -errno;
	}

	closedir(probe);

	WalkResult result{erase, 0, 0};
	walk(path, sizeof(path), max_depth, result);

	entries = result.entries;

	return result.failures > 0 ? -EIO : 0;
}

int count_logs(unsigned max_depth)
{
	int entries = 0;
	const int ret = run_walk(false, max_depth, entries);

	// Counting tolerates unreadable corners: report what was found.
	if (ret == -EIO) {
		return entries;
	}

	return ret < 0 ? ret : entries;
}

int erase_all_logs(unsigned max_depth)
{
	int entries = 0;
	const int ret = run_walk(true, max_depth, entries);

	if (ret < 0) {
		PX4_ERR("erase of %s incomplete: %d entries removed", kLogRoot, entries);
		return ret;
	}

	PX4_INFO("erased %d entries below %s", entries, kLogRoot);

	return entries;
}

} // namespace log_utils