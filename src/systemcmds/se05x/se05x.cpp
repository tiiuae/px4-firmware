/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/crypto/se05x.h>

static void usage()
{
	PRINT_MODULE_DESCRIPTION("Read the identity of the SE05x secure element");
	PRINT_MODULE_USAGE_NAME("se05x", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Print the unique id and the OEF id");
}

extern "C" __EXPORT int se05x_main(int argc, char *argv[])
{
	if (argc != 2 || strcmp(argv[1], "info") != 0) {
		usage();
		return 1;
	}

	int fd = open("/dev/se05x", O_RDWR);

	if (fd < 0) {
		PX4_ERR("/dev/se05x: %d", errno);
		return 1;
	}

	struct se05x_uid_s uid {};
	struct se05x_info_s info {};
	int ret = 1;

	if (ioctl(fd, SEIOC_GET_UID, (unsigned long)&uid) < 0) {
		PX4_ERR("unique id: %d", errno);

	} else if (ioctl(fd, SEIOC_GET_INFO, (unsigned long)&info) < 0) {
		PX4_ERR("OEF id: %d", errno);

	} else {
		PX4_INFO_RAW("unique id: ");

		for (size_t i = 0; i < sizeof(uid.uid); i++) {
			PX4_INFO_RAW("%02x", uid.uid[i]);
		}

		PX4_INFO_RAW("\nOEF id: 0x%04x\n", info.oef_id);
		ret = 0;
	}

	close(fd);
	return ret;
}
