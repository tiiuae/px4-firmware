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

#include "MoiAgent.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/shutdown.h>
#include <sys/time.h>


using namespace px4::moi_agent;
using namespace time_literals;


int moi_agent_main(int argc, char *argv[])
{
	return MoiAgent::main(argc, argv);
}

namespace px4
{
namespace moi_agent
{

static const char *state_long(const char state)
{
	switch (state) {
	case 'I':
		return "init";

	case 'M':
		return "maintenance";

	case 'O':
		return "operational";

	default:
		return "unknown";
	}
}

static char state_short(const char *state)
{
	if (strcmp(state, "init") == 0) {
		return 'I';

	} else if (strcmp(state, "maintenance") == 0) {
		return 'M';

	} else if (strcmp(state, "operational") == 0) {
		return 'O';

	} else {
		return '\0';
	}
}

static int read_state_from_storage(struct moi_store *store)
{
	FILE *fp;
	int cab;

	fp = fopen("/fs/microsd/moi_state.txt", "r");

	if (fp == NULL) {
		return PX4_ERROR;
	}

	fscanf(fp, "%c %d", &store->state, &cab);
	fclose(fp);

	if (cab == 0) {
		store->critical_activity_block = false;

	} else {
		store->critical_activity_block = true;
	}

	return PX4_OK;
}

static void write_state_to_storage(struct moi_store *store)
{
	FILE *fp;
	fp = fopen("/fs/microsd/moi_state.txt", "w");

	if (fp == NULL) {
		PX4_ERR("Failed to open M-O-I state file for writing");
		return;
	}

	fprintf(fp, "%c %d", store->state, store->critical_activity_block);
	fclose(fp);
}

MoiAgent::MoiAgent(uint16_t local_port) :
	ModuleParams(nullptr),
	_local_port(local_port)
{
	pthread_mutex_init(&_ca_mtx, nullptr);
}

MoiAgent::~MoiAgent()
{
	if (_listenfd >= 0) {
		close(_listenfd);
	}

	if (_connfd >= 0) {
		close(_connfd);
	}

	if (_json_recv.buf != nullptr) {
		free(_json_recv.buf);
	}

	if (_json_recv.json != nullptr) {
		free(_json_recv.json);
	}

	PX4_INFO("M-O-I agent destroyed");
	int ret = pthread_join(_critical_action_listener_thread, nullptr);

	if (ret) {
		PX4_ERR("listener thread join failed: %d", ret);
	}

	pthread_mutex_destroy(&_ca_mtx);
}

int MoiAgent::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("moi_agent",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      PX4_STACK_ADJUSTED(2000),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return 0;
}

MoiAgent *MoiAgent::instantiate(int argc, char *argv[])
{
	uint16_t local_port{MOI_AGENT_PRT};
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':

			local_port = (uint16_t)(strtoul(myoptarg, nullptr, 10) & 0xffff);
			break;

		case '?':

			PX4_WARN("option not supported");
			break;

		default:

			PX4_WARN("option not supported");
			break;
		}
	}

	MoiAgent *agent = new MoiAgent(local_port);

	if (agent == nullptr) {
		PX4_ERR("MoiAgent object alloc failed");
	}

	return agent;
}

void MoiAgent::run()
{
	struct moi_store store;

	// Start critical action request listener

	pthread_attr_t thr_attr;
	pthread_attr_init(&thr_attr);
	pthread_attr_setstacksize(&thr_attr, PX4_STACK_ADJUSTED(1500));
	PX4_INFO("create _critical_action_listener_thread");
	int ret = pthread_create(&_critical_action_listener_thread, &thr_attr, &MoiAgent::critical_action_listener_helper,
				 this);

	if (ret) {
		PX4_ERR("_critical_action_listener_thread create failed: %d", ret);
	}

	pthread_attr_destroy(&thr_attr);

	// Read state from file during startup

	if (read_state_from_storage(&store) == PX4_OK) {
		// Set current state
		_current_state = store.state;

		// mutex lock not needed as listener thread not yet started
		//_block_new_critical_activities = store.critical_activity_block;

		// Because reboot is performed in the end of state change, the
		// critical activity block can be released here
		_block_new_critical_activities = false;
	}

	if (_current_state == '\0') {
		PX4_ERR("Failed to read M-O-I state from internal storage");
		exit_and_cleanup();
		return;
	}

	/*
		PX4_INFO("*******************************");
		PX4_INFO("****** M-O-I state: '%c' *******", _current_state);
		PX4_INFO("****** Crit act: %s ******",
			 (_block_new_critical_activities) ? "blocked" : "allowed");
		PX4_INFO("*******************************");
	*/
	// Reserve receive buffer

	_json_recv.buf = (char *) malloc(MOI_AGENT_RECV_BUF_LEN);

	if (_json_recv.buf == nullptr) {
		PX4_ERR("Failed to allocate receive buffer");
		exit_and_cleanup();
		return;
	}

	_json_recv.json = (char *) malloc(MOI_AGENT_JSON_DATA_BUF_LEN);

	if (_json_recv.json == nullptr) {
		PX4_ERR("Failed to allocate json data buffer");
		exit_and_cleanup();
		return;
	}

	// Initialize cJSON hooks
	cJSON_InitHooks(NULL);

	run_loop();
}

void MoiAgent::run_loop()
{
	const int one = 1;
	int ret;
	bool connected = false;
	//int print_counter = 10;
	struct timeval tv = {
		// 200 ms
		0, 200000
	};

	// Create socket

	_listenfd = socket(PF_INET, SOCK_STREAM, 0);

	if (_listenfd < 0) {
		PX4_ERR("ERROR opening listen socket");
		exit_and_cleanup();
		return;
	}

	setsockopt(_listenfd, SOL_SOCKET, SO_REUSEADDR, &one, (socklen_t)sizeof(one));
	setsockopt(_listenfd, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv, sizeof(struct timeval));

	_listenaddr.sin_family = AF_INET;
	inet_pton(AF_INET, "0.0.0.0", &(_listenaddr.sin_addr)); // listen on all network interfaces
	_listenaddr.sin_port = htons(_local_port);

	memset(&_connaddr, 0, sizeof(_connaddr));

	PX4_INFO("Bind M-O-I agent to port %d", _local_port);
	ret = bind(_listenfd, (struct sockaddr *)&_listenaddr, sizeof(_listenaddr));

	if (ret == -1) {
		PX4_ERR("ERROR binding the listen socket");
		close(_listenfd);
		exit_and_cleanup();
		return;
	}

	_listen_fds[0].fd = _listenfd;
	_listen_fds[0].events = POLLIN;

	listen(_listenfd, 1);
	socklen_t connaddr_size = sizeof(_connaddr);

	PX4_INFO("Trying to connect M-O-I server..");

	while (true) {

		if (should_exit()) {
			exit_and_cleanup();
			break;
		}

		// Waiting connection from server

		_connfd = accept(_listenfd, (sockaddr *)&_connaddr, &connaddr_size);

		if (_connfd < 0) {
			/*
			if (!(--print_counter)) {
				PX4_INFO("Still waiting for M-O-I server connection..");
				print_counter = 20;
			}
			*/
			PX4_INFO("Accepting failed, retrying..");
			continue;
		}

		setsockopt(_connfd, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv, sizeof(struct timeval));
		connected = true;
		PX4_INFO("M-O-I server connected from %s:%d", inet_ntoa(_connaddr.sin_addr), ntohs(_connaddr.sin_port));

		// Set up poll for new connection


		while (connected) {

			if (should_exit() || _reboot_request == RebootState::REQUESTED) {

				// Close socket

				close(_connfd);
				_connfd = -1;

				// Let outer loop to handle exit_and_cleanup

				break;
			}

			ret = recv(_connfd, _json_recv.buf + _json_recv.wi, MOI_AGENT_RECV_BUF_LEN - _json_recv.wi, 0);

			if (ret < 0) {
				if (errno == ENOTCONN) {
					PX4_ERR("M-O-I server closed connection");
					connected = false;

				} else if (errno != EAGAIN) {
					PX4_ERR("ERROR in recv: %d", errno);
				}

			} else if (ret == 0) {
				PX4_ERR("M-O-I server closed connection");
				connected = false;

			} else {
				_json_recv.wi += ret;
				bool parsing = true;

				while (parsing) {
					if (_json_recv.wi < _json_recv.ri) {
						PX4_ERR("ERROR: wi < ri");
						_json_recv.wi = 0;
						_json_recv.ri = 0;
						break;
					}

					uint32_t datalen = _json_recv.wi - _json_recv.ri;
					//PX4_INFO("[PARSING]: index (ri, wi): %d .. %d (l:%d)", _json_recv.ri, _json_recv.wi, datalen);

					if (_json_recv.ri > MOI_AGENT_RECV_THRESHOLD) {
						//PX4_INFO("[THRESHOLD]: Read pointer reached the recv buffer threshold");
						if (datalen > 0) {
							// Move remaining data to the beginning of the buffer
							//PX4_INFO("[THRESHOLD]: Move remaining to the beginning....");
							memcpy(_json_recv.buf, _json_recv.buf + _json_recv.ri, _json_recv.wi - _json_recv.ri);
							_json_recv.wi -= _json_recv.ri;

						} else {
							_json_recv.wi = 0;
						}

						_json_recv.ri = 0;
						//PX4_INFO("[THRESHOLD]: index (ri, wi): %d .. %d (l:%d)", _json_recv.ri, _json_recv.wi, datalen);

					}

					switch (_json_recv.state) {
					case JsonState::LENGTH: {
							if (datalen > 3) {
								// Receiving new frame
								/*
								PX4_INFO("==== NEW FRAME ====");
								PX4_INFO("[LENGTH]: data: %c%c%c%c%c%c...",
									_json_recv.buf[_json_recv.ri],
									_json_recv.buf[_json_recv.ri+1],
									_json_recv.buf[_json_recv.ri+2],
									_json_recv.buf[_json_recv.ri+3],
									_json_recv.buf[_json_recv.ri+4],
									_json_recv.buf[_json_recv.ri+5]);
								PX4_INFO("[LENGTH]: data: ...%c%c%c%c%c%c",
									_json_recv.buf[_json_recv.wi-6],
									_json_recv.buf[_json_recv.wi-5],
									_json_recv.buf[_json_recv.wi-4],
									_json_recv.buf[_json_recv.wi-3],
									_json_recv.buf[_json_recv.wi-2],
									_json_recv.buf[_json_recv.wi-1]);
								*/

								_json_recv.json_len = ((uint32_t) _json_recv.buf[_json_recv.ri]) |
										      ((uint32_t) _json_recv.buf[_json_recv.ri + 1] << 8) |
										      ((uint32_t) _json_recv.buf[_json_recv.ri + 2] << 16) |
										      ((uint32_t) _json_recv.buf[_json_recv.ri + 3] << 24);
								_json_recv.ri += sizeof(uint32_t);
								//PX4_INFO("[LENGTH]: json_len: %d, ri: %d", _json_recv.json_len, _json_recv.ri);
								_json_recv.state = JsonState::DATA;

							} else {
								parsing = false;
							}
						}
						break;

					case JsonState::DATA: {
							if (datalen >= _json_recv.json_len) {
								//PX4_INFO("[DATA]: frame completely received: wi: %d, ri: %d, len:%d", _json_recv.wi, _json_recv.ri,
								//	_json_recv.json_len);

								// Copy json string into json buffer
								memcpy(_json_recv.json, _json_recv.buf + _json_recv.ri, _json_recv.json_len);
								// Null terminate json string
								_json_recv.json[_json_recv.json_len] = '\0';
								_json_recv.ri += _json_recv.json_len;

								//PX4_INFO("[DATA]: process: json_len: %d",_json_recv.json_len);
								char *resp = process_message(_json_recv.json);

								if (resp) {
									int len = strlen(resp);
									//PX4_INFO("[DATA]: process: resp (l:%d): '%s' ",len, resp);
									_json_recv.json[0] = len & 0xff;
									_json_recv.json[1] = (len >> 8) & 0xff;
									_json_recv.json[2] = (len >> 16) & 0xff;
									_json_recv.json[3] = (len >> 24) & 0xff;
									memcpy(_json_recv.json + 4, resp, len);

									ret = send(_connfd, _json_recv.json, len + 4, 0);

									if (ret < 0) {
										PX4_ERR("ERROR in send: %d", errno);
									}

									free(resp);
								}

								// Set frame completely received
								_json_recv.state = JsonState::LENGTH;

							} else {
								parsing = false;
							}
						}
						break;

					default: {
							PX4_ERR("Invalid state: %d", _json_recv.state);
							_json_recv.state = JsonState::LENGTH;
						}
						break;
					}
				}
			}

			px4_usleep(100_ms);
		} // while (connected);

		if (_reboot_request == RebootState::REQUESTED) {
			PX4_INFO("Reboot requested.....");
			ret = px4_reboot_request(false, 0, false);

			if (ret < 0) {
				PX4_ERR("reboot failed (%i)", ret);
			}
		}

	} //  while (true)

	close(_listenfd);
	_listenfd = -1;
	PX4_INFO("EXIT M-O-I agent");
}

void *MoiAgent::critical_action_listener_helper(void *context)
{
	px4_prctl(PR_SET_NAME, "moi_ca_listener", px4_getpid());
	static_cast<MoiAgent *>(context)->critical_action_listener();
	return nullptr;
}

void MoiAgent::critical_action_listener()
{
	while (true) {
		vehicle_command_s moi_critical_req;

		while (_crit_act_req_sub.update(&moi_critical_req)) {
			//PX4_INFO("[listener] updated");
			vehicle_command_ack_s moi_critical_resp{};
			moi_critical_resp.command = moi_critical_req.command;
			moi_critical_resp.timestamp = hrt_absolute_time();

			switch (moi_critical_req.command) {

			// Reserve critical action lock
			case vehicle_command_s::VEHICLE_CMD_CUSTOM_1: {
					PX4_INFO("[listener] Reqest to reserve critical action lock..");
					pthread_mutex_lock(&_ca_mtx);

					if (_block_new_critical_activities) {
						PX4_INFO("[listener] Critical activity block active => REJECT");
						moi_critical_resp.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

					} else {
						PX4_INFO("[listener] Critical activity allowed => ACCEPT");
						moi_critical_resp.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
						_critical_action_count++;
					}

					pthread_mutex_unlock(&_ca_mtx);
					_crit_act_resp_pub.publish(moi_critical_resp);
				}
				break;

			// Release critical action lock
			case vehicle_command_s::VEHICLE_CMD_CUSTOM_0: {
					PX4_INFO("[listener] Release critical action lock");
					moi_critical_resp.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

					pthread_mutex_lock(&_ca_mtx);
					_critical_action_count--;

					if (_critical_action_count < 0) {
						PX4_ERR("[listener] Critical action reserve/release out-of-sync!");
						_critical_action_count = 0;
					}

					pthread_mutex_unlock(&_ca_mtx);
					_crit_act_resp_pub.publish(moi_critical_resp);
				}
				break;

			default:
				break;

			}

		}

		px4_usleep(1000);
	}

	PX4_INFO("[listener] moi_critical_action_listener exiting");
}

char *MoiAgent::process_message(char *data)
{
	cJSON *req{nullptr};
	cJSON *req_op{nullptr};
	cJSON *resp{nullptr};
	char *resp_str{nullptr};

	// Parse request

	req = cJSON_Parse(data);

	if (!req) {
		PX4_ERR("Invalid JSON message: '%s'", cJSON_GetErrorPtr());
		return NULL;
	}

	// Handle request

	req_op = cJSON_GetObjectItem(req, "Op");

	if (!req_op || !req_op->valuestring) {
		PX4_ERR("Missing 'Op' field");
		cJSON_Delete(req);
		return NULL;
	}

	if (strcmp(req_op->valuestring, "CHECK_STATE_REQUEST") == 0) {
		resp = handle_check_state_request(req);

	} else if (strcmp(req_op->valuestring, "READINESS_STATUS_REQUEST") == 0) {
		resp = handle_readiness_status_request(req);

	} else if (strcmp(req_op->valuestring, "CHANGE_STATE_REQUEST") == 0) {
		resp = handle_change_state_request(req);

	} else if (strcmp(req_op->valuestring, "CHANGE_STATE_COMPLETE_IND") == 0) {
		resp = handle_change_state_complete_ind(req);

	} else {
		PX4_ERR("Unknown command : '%s'", req_op->valuestring);
		cJSON_Delete(req);
		return NULL;
	}

	if (resp) {
		resp_str = cJSON_Print(resp);
		cJSON_Delete(resp);
	}

	cJSON_Delete(req);
	return resp_str;
}

cJSON *MoiAgent::handle_check_state_request(cJSON *root)
{
	cJSON *cs{nullptr};

	// Generate response

	cJSON *resp_root = cJSON_CreateObject();
	cJSON_AddStringToObject(resp_root, "Op", "CHECK_STATE_RESPONSE");
	cJSON_AddItemToObject(resp_root, "CheckStateResponse", cs = cJSON_CreateObject());
	cJSON_AddStringToObject(cs, "CurrentState", state_long(_current_state));

	return resp_root;
}

cJSON *MoiAgent::handle_readiness_status_request(cJSON *root)
{
	char new_state;
	char assumed;
	cJSON *rsr{nullptr};
	cJSON *acs{nullptr};
	cJSON *ns{nullptr};

	rsr = cJSON_GetObjectItem(root, "ReadinessStatusRequest");

	if (!rsr) {
		PX4_ERR("Missing 'ReadinessStatusRequest' field");
		return create_readiness_status_resp("Missing 'ReadinessStatusRequest' field");
	}

	// Parse assumed_state

	acs = cJSON_GetObjectItem(rsr, "AssumedCurrentState");
	assumed = json_parse_state(acs);

	if (assumed == '\0') {
		PX4_ERR("Invalid 'AssumedCurrentState' field");
		return create_readiness_status_resp("Invalid 'AssumedCurrentState' field");
	}

	if (assumed != _current_state) {
		PX4_INFO("CHECKING: Agent states out of sync: assumed %c, current: %c", assumed, _current_state);
	}


	// Parse new_state

	ns = cJSON_GetObjectItem(rsr, "NewState");
	new_state = json_parse_state(ns);

	if (new_state == '\0') {
		PX4_ERR("Invalid 'NewState' field");
		return create_readiness_status_resp("Invalid 'NewState' field");
	}

	pthread_mutex_lock(&_ca_mtx);

	if (_critical_action_count) {
		pthread_mutex_unlock(&_ca_mtx);
		PX4_INFO("Critical activity ongoing, deny state change");
		return create_readiness_status_resp("FC has critical activity");
	}

	PX4_INFO("CHECKING: Assumed: %c, New: %c", assumed, new_state);
	_requested_state = new_state;
	set_crit_act_block(true);
	PX4_INFO("Critical activity blocked!");
	pthread_mutex_unlock(&_ca_mtx);

	return create_readiness_status_resp("");
}

cJSON *MoiAgent::handle_change_state_request(cJSON *root)
{
	char state = _current_state;
	cJSON *sr{nullptr};
	cJSON *to{nullptr};

	sr = cJSON_GetObjectItem(root, "ChangeStateRequest");

	if (!sr) {
		PX4_ERR("Missing 'ChangeStateRequest' field");
		return create_change_state_resp(state, "Missing 'ChangeStateRequest' field");
	}

	// Parse To state

	to = cJSON_GetObjectItem(sr, "To");
	state = json_parse_state(to);

	if (state == '\0') {
		PX4_ERR("Invalid 'To' field");
		return create_change_state_resp(state, "Invalid 'To' field");
	}

	if (state != _current_state) {
		set_state(state);
		_reboot_request = RebootState::PREPARED;

	} else {
		PX4_INFO("No state change");
	}

	return create_change_state_resp(state, "");
}

cJSON *MoiAgent::handle_change_state_complete_ind(cJSON *root)
{
	//char new_state;
	//cJSON *csci{nullptr};
	//cJSON *ns{nullptr};

	/*
		pthread_mutex_lock(&_ca_mtx);

		if (!_block_new_critical_activities) {
			PX4_ERR("Agent out of sync: received unexpected 'ChangeStateCompleteInd'");
		}

		PX4_INFO("Critical activity block released!");
		set_crit_act_block(false);
		pthread_mutex_unlock(&_ca_mtx);
	*/
	if (_reboot_request == RebootState::PREPARED) {
		_reboot_request = RebootState::REQUESTED;
	}

	/*
		csci = cJSON_GetObjectItem(root, "ChangeStateCompleteInd");

		if (!csci) {
			PX4_ERR("Missing 'ChangeStateCompleteInd' field");
			return create_change_state_complete_resp();
		}

		// Parse NewState

		ns = cJSON_GetObjectItem(csci, "State");
		new_state = json_parse_state(ns);

		if (new_state == '\0') {
			PX4_ERR("Invalid 'State' field");
			return create_change_state_complete_resp();
		}

		if (new_state != _current_state) {
			PX4_ERR("Agent states out of sync: assumed %c, current: %c", new_state, _current_state);
		}
	*/
	return create_change_state_complete_resp();
}

cJSON *MoiAgent::create_readiness_status_resp(const char *error)
{
	cJSON *resp_root{nullptr};
	cJSON *resp_rsr{nullptr};

	resp_root = cJSON_CreateObject();
	cJSON_AddStringToObject(resp_root, "Op", "READINESS_STATUS_RESPONSE");
	cJSON_AddItemToObject(resp_root, "ReadinessStatusResponse", resp_rsr = cJSON_CreateObject());
	cJSON_AddStringToObject(resp_rsr, "Error", error);
	return resp_root;
}

cJSON *MoiAgent::create_change_state_resp(char state, const char *error)
{
	cJSON *resp_root{nullptr};
	cJSON *resp_sr{nullptr};

	resp_root = cJSON_CreateObject();
	cJSON_AddStringToObject(resp_root, "Op", "CHANGE_STATE_RESPONSE");
	cJSON_AddItemToObject(resp_root, "ChangeStateResponse", resp_sr = cJSON_CreateObject());
	cJSON_AddStringToObject(resp_sr, "NewState", state_long(state));
	cJSON_AddStringToObject(resp_sr, "Error", error);

	return resp_root;
}

cJSON *MoiAgent::create_change_state_complete_resp()
{
	cJSON *resp_root{nullptr};

	resp_root = cJSON_CreateObject();
	cJSON_AddStringToObject(resp_root, "Op", "EMPTY_RESPONSE");
	return resp_root;
}

bool MoiAgent::check_readiness()
{
	// Check if FC is ready to change state

	if (_requested_state == 'I') {
		// Always allow to go to init state
		return true;
	}

	switch (_current_state) {
	case 'I':
		return true;

	case 'M':
		if (check_is_calibration() || check_is_fw_update()) {
			return false;
		}

		return true;

	case 'O':
		if (check_is_airborne()) {
			return false;
		}

		return true;

	default:
		PX4_INFO("Unknown current state: %c", _current_state);
		return true;
	}

	return true;
}

bool MoiAgent::check_is_calibration()
{
	return false;
}
bool MoiAgent::check_is_fw_update()
{
	return false;
}
bool MoiAgent::check_is_airborne()
{
	return false;
}

char MoiAgent::json_parse_state(cJSON *item)
{
	if (!item || !item->valuestring) {
		return '\0';
	}

	return state_short(item->valuestring);
}

void MoiAgent::set_crit_act_block(bool block)
{
	// _ca_mtx Mutex shall be already locked while calling this function
	struct moi_store store;
	_block_new_critical_activities = block;

	if (read_state_from_storage(&store) == PX4_ERROR) {
		PX4_ERR("current M-O-I state not found from internal storage");
		store.state = _current_state;
	}

	store.critical_activity_block = _block_new_critical_activities;
	write_state_to_storage(&store);
}

int MoiAgent::set_state(char state)
{
	struct moi_store store;

	if (state != 'M' && state != 'O' && state != 'I') {
		PX4_ERR("Invalid M-O-I state");
		return PX4_ERROR;
	}

	if (read_state_from_storage(&store) == PX4_ERROR) {
		PX4_INFO("current M-O-I state not found from internal storage");
		store.critical_activity_block = _block_new_critical_activities;

	} else if (state == store.state) {
		PX4_INFO("M-O-I state already set to '%c'", state);
		return PX4_OK;
	}

	store.state = state;
	write_state_to_storage(&store);

	return PX4_OK;
}

char MoiAgent::get_current_state()
{
	return _current_state;
}

int MoiAgent::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("M-O-I-agent not running");
		return 1;
	}

	if (!strcmp(argv[0], "cmp")) {
		if (argc < 2) {
			print_usage("missing argument");
			return 1;
		}

		if (get_instance()->get_current_state() == ((char) argv[1][0])) {
			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[0], "set")) {
		if (argc < 2) {
			print_usage("missing argument");
			return 1;
		}

		if (get_instance()->set_state((char) argv[1][0]) != PX4_OK) {
			PX4_ERR("M-O-I-agent state change failed");
			return 1;
		}

		return 0;
	}

	return print_usage("unknown command");
}

int MoiAgent::print_status()
{
	PX4_INFO("running. M-O-I state: '%c'", _current_state);
	return 0;
}

int MoiAgent::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
M-O-I state agent. This module is responsible for handling the M-O-I state of the FC subsystem. It is started by the M-O-I-agent module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("moi_agent", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("cmp", "Compare current M-O-I state with string. Returns 0 on match. Otherwise returns 1");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Set current M-O-I state. Allowed states are 'I', 'M' and 'O'");
	PRINT_MODULE_USAGE_PARAM_INT('p', -1, 0, 65535, "Agent listening port. If not provided, defaults to MOI_AGENT_PRT", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

} // namespace moi_agent
} // namespace px4
