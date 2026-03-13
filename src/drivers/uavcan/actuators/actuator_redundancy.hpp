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
 * @file actuator_redundancy.hpp
 *
 * Actuator redundancy helper functions for UAVCAN
 *
 * @author Jukka Laitinen <jukka.laitinen@tii.ae>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/redundancy_status.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

static constexpr int MaxNFCs = vehicle_status_s::MAX_REDUNDANT_CONTROLLERS;

/**
 * Generic actuator redundancy helper for UAVCAN
 * @tparam MessageType - UAVCAN message type to monitor (e.g., uavcan::equipment::esc::RawCommand or uavcan::equipment::actuator::ArrayCommand)
 */
template<typename MessageType>
class ActuatorRedundancy
{
public:
	ActuatorRedundancy(uavcan::INode &node);
	~ActuatorRedundancy();

	int Init();
	bool Enabled() const { return _redundant_actuator_control_enabled; }
	bool RelayActiveOutputs(uint16_t *outputs, unsigned num_outputs, bool &stop_motors);

private:
	using CallbackBinder = uavcan::MethodBinder<ActuatorRedundancy<MessageType> *,
	      void (ActuatorRedundancy<MessageType>::*)(const uavcan::ReceivedDataStructure<MessageType>&)>;

	uavcan::INode &_node;
	uavcan::Subscriber<MessageType, CallbackBinder> _uavcan_sub_cmd;

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _redundancy_status_sub{ORB_ID(redundancy_status)};
	uORB::Subscription *_redundant_actuator_outputs_sub[MaxNFCs] {nullptr};

	bool _redundant_actuator_control_enabled{false};
	hrt_abstime _last_other_fc_cmd_time{0};	///< Last time we heard command from other FC on the bus

	void deleteSubscriptions();

	/**
	 * Message reception callback (for monitoring other FC's CAN traffic)
	 */
	void message_sub_cb(const uavcan::ReceivedDataStructure<MessageType> &msg);
};

// Include implementation for template class
#include "actuator_redundancy_impl.hpp"
