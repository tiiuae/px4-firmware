/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file actuator_redundancy_impl.hpp
 *
 * Actuator redundancy helper template implementation for UAVCAN
 *
 * @author Jukka Laitinen <jukka.laitinen@tii.ae>
 */

#pragma once

#include <parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

using namespace time_literals;

#ifdef CONFIG_MODULES_REDUNDANCY

template<typename MessageType>
ActuatorRedundancy<MessageType>::ActuatorRedundancy(uavcan::INode &node)
	: _node(node)
	, _uavcan_sub_cmd(node)
{
}

template<typename MessageType>
ActuatorRedundancy<MessageType>::~ActuatorRedundancy()
{
	deleteSubscriptions();
}

template<typename MessageType>
int
ActuatorRedundancy<MessageType>::Init()
{
	int32_t spare_autopilots;

	if (param_get(param_find("FT_N_SPARE_FCS"), &spare_autopilots) == PX4_OK && spare_autopilots > 0
	    && spare_autopilots < MaxNFCs) {
		/* Start command subscriber to monitor other FC's CAN traffic */
		int res = _uavcan_sub_cmd.start(CallbackBinder(this, &ActuatorRedundancy<MessageType>::message_sub_cb));

		if (res < 0) {
			PX4_ERR("command subscriber start failed %i", res);
			return res;
		}

		/* Find out which actuator_outputs instance we are publishing.
		 * Note: there is a race condition in here in theory; if
		 * drivers publishing actuator outputs are started in parallel
		 * threads, this might give wrong results. However, in practice
		 * all the actuator drivers are started at boot in a single
		 * thread sequentially.
		 */

		int actuator_output_instance = orb_group_count(ORB_ID(actuator_outputs)) - 1;

		/* Sanity check; this only supports 4 instances, since the current
		 * redundancy communication interface (mavlink) doesn't support
		 * sharing more
		 */

		if (actuator_output_instance < 0 || actuator_output_instance > 3) {
			return PX4_ERROR;
		}

		/* Subscribe to the correct actuator outputs from the other FCs;
		 * this assumes that the same actuator control drivers are running
		 * on all FCs; that is, the topic instance numbers match.
		 * There is typically one instance for UAVCAN and another for PWM_ESC.
		 */

		for (int i = 0; i < MaxNFCs; i++) {
			const orb_metadata *meta;

			switch (i + redundancy_status_s::FC1) {
			case redundancy_status_s::FC1:
				meta = ORB_ID(redundant_actuator_outputs0);
				break;

			case redundancy_status_s::FC2:
				meta = ORB_ID(redundant_actuator_outputs1);
				break;

			case redundancy_status_s::FC3:
				meta = ORB_ID(redundant_actuator_outputs2);
				break;

			case redundancy_status_s::FC4:
				meta = ORB_ID(redundant_actuator_outputs3);
				break;

			default:
				return PX4_ERROR;
			}

			_redundant_actuator_outputs_sub[i] = new uORB::Subscription(meta, actuator_output_instance);

			if (!_redundant_actuator_outputs_sub[i]) {
				PX4_ERR("redundant_actuator_outputs%d not available", i);
				deleteSubscriptions();
				return PX4_ERROR;
			}
		}

		_redundant_actuator_control_enabled = true;
	}

	return PX4_OK;
}

template<typename MessageType>
bool
ActuatorRedundancy<MessageType>::RelayActiveOutputs(uint16_t *outputs, unsigned num_outputs, bool &stop_motors)
{
	/* For UAVCAN actuator redundancy:
	 * - Only send CAN commands if this FC is armed
	 * - If another FC is in control but not sending on CAN bus, relay their actuator outputs.
	 *   This ensures there are always actuator commands on the bus when armed.
	 * - If we hear the other FC on CAN bus, don't send anything
	 * As a sanity check, verify that the redundancy status and any redundant actuator
	 * outputs are less than 50 ms old.
	 *
	 * @return true if commands should be sent on the bus, false otherwise
	 */

	if (_redundant_actuator_control_enabled) {
		actuator_armed_s actuator_armed;
		redundancy_status_s rstatus;
		bool rstatus_available = _redundancy_status_sub.copy(&rstatus);

		if ((!_actuator_armed_sub.copy(&actuator_armed) || !actuator_armed.armed) &&
		    (!rstatus_available || hrt_elapsed_time(&rstatus.timestamp) >= 50_ms ||
		     rstatus.fc_number != rstatus.fc_in_act_control)) {
			/* We are disarmed and (not in actuator control or don't know about the other FC).
			 * Just don't send anything.
			 * Note: For a better redundancy support, the ESCs would support receiving data from
			 *       multiple FCs, and the switching logic (timeout) would be implemented there.
			 *       in this case, we most likely wouldn't need this.
			 */
			return false;
		}

		/* Check if another FC is in control */
		if (rstatus_available &&
		    hrt_elapsed_time(&rstatus.timestamp) < 50_ms &&
		    rstatus.fc_number != rstatus.fc_in_act_control &&
		    rstatus.fc_number >= redundancy_status_s::FC1 &&
		    rstatus.fc_number < redundancy_status_s::FC1 + MaxNFCs &&
		    rstatus.fc_in_act_control >= redundancy_status_s::FC1 &&
		    rstatus.fc_in_act_control < redundancy_status_s::FC1 + MaxNFCs) {
			/* Another FC is in control, check if we hear it on the CAN bus
			 * Note: For a better redundancy support, the ESCs would support receiving data from
			 *       multiple FCs, and the switching logic (timeout) would be implemented there.
			 *       in this case, we should just always send the actuator outputs here.
			 */
			if (hrt_elapsed_time(&_last_other_fc_cmd_time) < 50_ms) {
				/* Yes, don't send anything */
				return false;
			}

			/* We don't hear the other FCs on the bus, but someone else is in control.
			 * Act as a relay and send the other FCs actuator outputs.
			 */

			int fc_idx = rstatus.fc_in_act_control - redundancy_status_s::FC1;
			actuator_outputs_s ract_outputs;

			if (_redundant_actuator_outputs_sub[fc_idx]->copy(&ract_outputs) &&
			    hrt_elapsed_time(&ract_outputs.timestamp) < 50_ms) {
				num_outputs = math::min(num_outputs, (unsigned)ract_outputs.noutputs);

				for (unsigned i = 0; i < num_outputs; i++) {
					outputs[i] = ract_outputs.output[i];
				}

				stop_motors = false;
			}
		}
	}

	return true;
}

template<typename MessageType>
void
ActuatorRedundancy<MessageType>::deleteSubscriptions()
{
	/* delete redundant_actuator_output subscriptions */
	for (int i = 0; i < MaxNFCs; i++) {
		if (_redundant_actuator_outputs_sub[i]) {
			delete _redundant_actuator_outputs_sub[i];
			_redundant_actuator_outputs_sub[i] = nullptr;
		}
	}
}

template<typename MessageType>
void
ActuatorRedundancy<MessageType>::message_sub_cb(const uavcan::ReceivedDataStructure<MessageType> &msg)
{
	/* Check if this message is from another FC */
	uint8_t src_node_id = msg.getSrcNodeID().get();
	uint8_t this_node_id = _node.getNodeID().get();

	if (src_node_id >= 1 && src_node_id <= MaxNFCs && src_node_id != this_node_id) {
		_last_other_fc_cmd_time = hrt_absolute_time();
	}
}

#endif // #ifdef CONFIG_MODULES_REDUNDANCY
