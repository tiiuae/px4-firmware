/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netutils/cJSON.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

extern "C" __EXPORT int moi_agent_main(int argc, char *argv[]);

const uint16_t MOI_AGENT_PRT = 4444;
const size_t   MOI_AGENT_RECV_BUF_LEN = 1024;
const size_t   MOI_AGENT_JSON_DATA_BUF_LEN = 512;

//const size_t   MOI_AGENT_RECV_THRESHOLD = MOI_AGENT_RECV_BUF_LEN-(MOI_AGENT_RECV_BUF_LEN/2);
const size_t   MOI_AGENT_RECV_THRESHOLD = 100;

namespace px4
{
namespace moi_agent
{

enum JsonState {
	LENGTH = 0,
	DATA,
};

struct moi_store {
	char state;
	bool critical_activity_block;
};

struct moi_json_recv {
	JsonState state{JsonState::LENGTH};
	uint32_t json_len{0};
	char *json{nullptr};
	char *buf{nullptr};
	uint32_t wi{0};
	uint32_t ri{0};
};

class MoiAgent : public ModuleBase<MoiAgent>, public ModuleParams
{
public:


	MoiAgent(uint16_t _local_port);
	~MoiAgent() override;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MoiAgent *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	char get_current_state();

	int set_state(char state);

	void set_thr(pthread_t *thr);

	static void *critical_action_listener_helper(void *context);
	void critical_action_listener();


private:

	void run_loop();
	char *process_message(char *data);
	char json_parse_state(cJSON *item);
	cJSON *handle_check_state_request(cJSON *root);
	cJSON *handle_readiness_status_request(cJSON *root);
	cJSON *handle_change_state_request(cJSON *root);
	cJSON *create_readiness_status_resp(const char *error);
	cJSON *create_change_state_resp(char state, const char *error);
	void handle_change_state_complete_ind(cJSON *root);
	void set_crit_act_block(bool block);
	void check_critical_requests();

	bool check_readiness();
	bool check_is_calibration();
	bool check_is_fw_update();
	bool check_is_airborne();

	struct moi_json_recv _json_recv;

	uint16_t _local_port{0};
	int _listenfd{-1};
	int _connfd{-1};
	struct pollfd _listen_fds[1];
	struct sockaddr_in _listenaddr {};
	struct sockaddr_in _connaddr {};

	char _current_state{'I'};
	char _requested_state{'I'};
	bool _reboot_request{false};
	bool _block_new_critical_activities{false};

	pthread_mutex_t _ca_mtx;
	int _critical_action_count{0};
	pthread_t _critical_action_listener_thread{0};

	uORB::Subscription _crit_act_req_sub{ORB_ID(moi_critical_req)};
	uORB::Publication<vehicle_command_ack_s> _crit_act_resp_pub{ORB_ID(moi_critical_resp)};
};

} //namespace moi_agent
} //namespace px4
