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
#include <stdio.h>

#ifdef __PX4_NUTTX
#define LOG_UTILS_DIRENT_DIR  DTYPE_DIRECTORY
#define LOG_UTILS_DIRENT_FILE DTYPE_FILE
#else
#define LOG_UTILS_DIRENT_DIR  DT_DIR
#define LOG_UTILS_DIRENT_FILE DT_REG
#endif

namespace log_utils
{

static const char *const kLogRoot = PX4_STORAGEDIR "/log";

const char *log_root()
{
	return kLogRoot;
}

static int count_in(const char *dir, unsigned depth_left)
{
	DIR *dp = opendir(dir);

	if (dp == nullptr) {
		return -errno;
	}

	int count = 0;
	struct dirent *entry = nullptr;

	while ((entry = readdir(dp)) != nullptr) {
		if (entry->d_type == LOG_UTILS_DIRENT_FILE) {
			count++;

		} else if (entry->d_type == LOG_UTILS_DIRENT_DIR && entry->d_name[0] != '.' && depth_left > 0) {
			char path[PATH_MAX];
			int len = snprintf(path, sizeof(path), "%s/%s", dir, entry->d_name);

			if (len > 0 && len < (int)sizeof(path)) {
				int sub = count_in(path, depth_left - 1);

				if (sub > 0) {
					count += sub;
				}

			} else {
				PX4_WARN("path too long, skipping %s/%s", dir, entry->d_name);
			}
		}
	}

	closedir(dp);

	return count;
}

int count_logs(unsigned max_depth)
{
	return count_in(kLogRoot, max_depth);
}

int erase_all_logs()
{
	// TODO(phase2): see the note in log_utils.h. Deliberately not implemented
	// so that an accidental call cannot destroy logs during bring-up.
	PX4_ERR("log_utils::erase_all_logs() is not implemented yet");
	return -ENOSYS;
}

} // namespace log_utils
