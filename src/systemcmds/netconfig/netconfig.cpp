/****************************************************************************
 *
 *   Copyright (c) 2023 Technology Innovation Institute. All rights reserved.
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
 * @file netconfig.cpp
 * Simple network configuration
 *
 * @author Jukka Laitinen
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <netinet/in.h>
#include <netutils/netlib.h>
#include <lib/parameters/param.h>
#include <uORB/topics/vehicle_status.h>

__BEGIN_DECLS
__EXPORT int  netconfig_main(int argc, char *argv[]);
__END_DECLS

int netconfig_main(int argc, char *argv[])
{
	struct in_addr addr;
	int32_t mav_id;
	int32_t mav_comp_id;
	int32_t ip0;
	int32_t ip1;
	const char ifname[] = CONFIG_NETCONFIG_IFNAME;

	param_get(param_find("MAV_SYS_ID"), &mav_id);
	param_get(param_find("MAV_COMP_ID"), &mav_comp_id);

	if (mav_id < 1 || mav_comp_id < 1 || mav_comp_id > vehicle_status_s::MAX_REDUNDANT_CONTROLLERS) {
		return PX4_ERROR;
	}

	/* IP: CONFIG_NETCONFIG_IPSUBNET + mav_id + mav_comp_id */

	addr.s_addr = CONFIG_NETCONFIG_IPSUBNET & 0xffff;

	/* Autopilot IP examples:
	   MAV_ID 1:
	   x.x.200.101 : primary FC1 (comp_id 1)
	   x.x.200.102 : redundant FC2 (comp_id 2)
	   x.x.200.103 : redundant FC3 (comp_id 3)
	   x.x.200,104 : redundant FC4 (comp_id 4)
	   MAV_ID 2:
	   x.x.201.101 : primary FC1 (comp_id 1)
	   x.x.201.102 : redundant FC2 (comp_id 2)
	   x.x.201.103 : redundant FC3 (comp_id 3)
	   x.x.201,104 : redundant FC4 (comp_id 4)
	*/

	ip1 = 200 + mav_id - 1;
	ip0 = 100 + mav_comp_id;

	if (ip0 > 253 || ip1 > 253) {
		return PX4_ERROR;
	}

	addr.s_addr |= (ip0 << 24) | (ip1 << 16);
	netlib_set_ipv4addr(ifname, &addr);

	/* GW */

	addr.s_addr = CONFIG_NETCONFIG_DRIPADDR;
	netlib_set_dripv4addr(ifname, &addr);

	/* netmask */

	addr.s_addr = CONFIG_NETCONFIG_NETMASK;
	netlib_set_ipv4netmask(ifname, &addr);

	netlib_ifup(ifname);

	return PX4_OK;
}
