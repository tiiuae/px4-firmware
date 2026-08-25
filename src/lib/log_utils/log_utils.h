/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file log_utils.h
 *
 * Shared helpers for inspecting and (eventually) erasing the onboard log
 * directory, so that more than one module can reach this logic.
 *
 * Today the erase walk lives as a private static member of MavlinkLogHandler
 * (src/modules/mavlink/mavlink_log_handler.cpp:607-653) and is reachable only
 * from the MAVLink LOG_ERASE handler. This library is where that walk moves to.
 */

#pragma once

namespace log_utils
{

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
 * @param max_depth how many directory levels to descend before giving up.
 *                  Guards against the unbounded recursion that the TODO at
 *                  mavlink_log_handler.cpp:631 flags in the original walk.
 * @return number of regular files found, or a negative errno on failure to
 *         open the log root (e.g. -ENOENT when no card is mounted)
 */
int count_logs(unsigned max_depth = 8);

/**
 * Erase every file and directory below log_root().
 *
 * TODO(phase2): not implemented yet -- currently returns -ENOSYS so that an
 * accidental early call fails loudly instead of deleting flight logs. Port the
 * recursive walk from mavlink_log_handler.cpp:607-653 here, adding:
 *   - the depth bound the TODO at :631 asks for,
 *   - a symlink guard,
 *   - a path buffer larger than the 128 bytes that currently causes long
 *     paths to be skipped silently.
 * Then repoint MavlinkLogHandler::_log_request_erase() at this function,
 * keeping its _current_status reset and _close_and_unlink_files() calls
 * (mavlink_log_handler.cpp:267-268) intact.
 *
 * @return 0 on success, negative errno on failure
 */
int erase_all_logs();

} // namespace log_utils
