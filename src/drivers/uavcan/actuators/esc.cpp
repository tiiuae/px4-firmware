/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file esc.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "esc.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanEscController::UavcanEscController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_uavcan_sub_status(node)
{
	_uavcan_pub_raw_cmd.setPriority(uavcan::TransferPriority::NumericallyMin); // Highest priority
	param_get(param_find("UAVCAN_ESC_RL"), &_uavcan_rate_limit_enable);
}

UavcanEscController::~UavcanEscController()
{
#ifdef CONFIG_MODULES_REDUNDANCY

	/* delete redundant_actuator_output subscriptions */
	for (int i = 0; i < MAX_N_FCS; i++) {
		delete _redundant_actuator_outputs_sub[i];
	}

#endif
}

int
UavcanEscController::init()
{
	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC status sub failed %i", res);
		return res;
	}

	_esc_status_pub.advertise();

#ifdef CONFIG_MODULES_REDUNDANCY
	int32_t spare_autopilots;

	if (param_get(param_find("FT_N_SPARE_FCS"), &spare_autopilots) == PX4_OK && spare_autopilots > 0
	    && spare_autopilots < MAX_N_FCS) {
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

		for (int i = 0; i < MAX_N_FCS; i++) {
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
				meta = nullptr;
			}

			_redundant_actuator_outputs_sub[i] = new uORB::Subscription(meta, actuator_output_instance);

			if (!_redundant_actuator_outputs_sub[i]) {
				PX4_ERR("redundant_actuator_outputs%d not available\n", i);
				return PX4_ERROR;
			}
		}

		_redundant_actuator_control_enabled = true;
	}

#endif

	return res;
}

void
UavcanEscController::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
#ifdef CONFIG_MODULES_REDUNDANCY
	/* For UAVCAN ESC redundancy:
	 * - Only send CAN commands if this FC is armed
	 * - If another FC is in control and we hear its actuator outputs on the CAN bus,
	 *   suppress our output completely (don't send anything)
	 * - This prevents both FCs from simultaneously sending commands with different CAN IDs
	 *
	 * As a sanity check, verify that the redundancy status and any redundant actuator
	 * outputs are less than 50 ms old.
	 */

	if (_redundant_actuator_control_enabled) {
		actuator_armed_s actuator_armed;
		redundancy_status_s rstatus;

		/* Check if this FC is armed */
		if (!_actuator_armed_sub.copy(&actuator_armed) || !actuator_armed.armed) {
			/* Not armed, don't send any commands */
			return;
		}

		/* Check if another FC is in control and actively sending on the CAN bus */
		if (_redundancy_status_sub.copy(&rstatus) &&
		    hrt_elapsed_time(&rstatus.timestamp) < 50_ms &&
		    rstatus.fc_number != rstatus.fc_in_act_control &&
		    rstatus.fc_number >= redundancy_status_s::FC1 &&
		    rstatus.fc_number < redundancy_status_s::FC1 + MAX_N_FCS &&
		    rstatus.fc_in_act_control >= redundancy_status_s::FC1 &&
		    rstatus.fc_in_act_control < redundancy_status_s::FC1 + MAX_N_FCS) {

			/* Another FC is in control, check if we hear it on the CAN bus */
			int fc_idx = rstatus.fc_in_act_control - redundancy_status_s::FC1;
			actuator_outputs_s ract_outputs;

			if (_redundant_actuator_outputs_sub[fc_idx]->copy(&ract_outputs) &&
			    hrt_elapsed_time(&ract_outputs.timestamp) < 50_ms) {
				/* We hear the controlling FC on the CAN bus, suppress our output */
				return;
			}
		}
	}

#endif

	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();

	if (_uavcan_rate_limit_enable == 1 && (timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	/*
	 * Fill the command message
	 * If unarmed, we publish an empty message anyway
	 */
	uavcan::equipment::esc::RawCommand msg;

	for (unsigned i = 0; i < num_outputs; i++) {
		if (stop_motors || outputs[i] == DISARMED_OUTPUT_VALUE) {
			msg.cmd.push_back(static_cast<unsigned>(0));

		} else {
			msg.cmd.push_back(static_cast<int>(outputs[i]));
		}
	}

	/*
	 * Remove channels that are always zero.
	 * The objective of this optimization is to avoid broadcasting multi-frame transfers when a single frame
	 * transfer would be enough. This is a valid optimization as the UAVCAN specification implies that all
	 * non-specified ESC setpoints should be considered zero.
	 * The positive outcome is a (marginally) lower bus traffic and lower CPU load.
	 *
	 * From the standpoint of the PX4 architecture, however, this is a hack. It should be investigated why
	 * the mixer returns more outputs than are actually used.
	 */
	for (int index = int(msg.cmd.size()) - 1; index >= _max_number_of_nonzero_outputs; index--) {
		if (msg.cmd[index] != 0) {
			_max_number_of_nonzero_outputs = index + 1;
			break;
		}
	}

	msg.cmd.resize(_max_number_of_nonzero_outputs);

	/*
	 * Publish the command message to the bus
	 * Note that for a quadrotor it takes one CAN frame
	 */
	_uavcan_pub_raw_cmd.broadcast(msg);
}

void
UavcanEscController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
}

void
UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.esc_index];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();
		ref.esc_voltage     = msg.voltage;
		//ref.esc_current     = fabs(msg.current);
		ref.esc_current     = 0.0; // currently with Myxa ESC, the reported current is not reliable
		ref.esc_temperature = msg.temperature;
		ref.esc_rpm         = msg.rpm;
		ref.esc_errorcount  = msg.error_count;

		_esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}
}

uint8_t
UavcanEscController::check_escs_status()
{
	int esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}
