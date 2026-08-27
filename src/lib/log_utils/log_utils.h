/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file log_utils.h
 *
 * Shared helpers for inspecting and erasing the onboard log directory, so that
 * more than one module can reach this logic.
 */

#pragma once

namespace log_utils
{

/**
 * Default recursion limit.
 */
static constexpr unsigned kDefaultMaxDepth{4};

/**
 * Absolute path of the log directory, i.e. PX4_STORAGEDIR "/log"
 * (/fs/microsd/log on NuttX targets).
 *
 * @return pointer to a static string, never null
 */
const char *log_root();

/**
 * Count the regular files below log_root(), recursing into session
 * directories. Read-only: opens directories, never modifies anything.
 *
 * @param max_depth directory levels to descend before giving up
 * @return number of regular files found, or a negative errno if the log root
 *         could not be opened (e.g. -ENOENT when no card is mounted)
 */
int count_logs(unsigned max_depth = kDefaultMaxDepth);

/**
 * Delete every file and directory below log_root(). The log root itself is
 * kept, so the logger can create new sessions without remounting.
 *
 * Symlinks are unlinked but never followed, so a link pointing outside the log
 * tree cannot cause deletion elsewhere.
 *
 * @param max_depth directory levels to descend before giving up
 * @return number of entries removed (>= 0) if everything below log_root() was
 *         removed successfully; -EIO if one or more entries could not be
 *         removed (details are logged); or another negative errno if the log
 *         root could not be opened at all
 */
int erase_all_logs(unsigned max_depth = kDefaultMaxDepth);

} // namespace log_utils