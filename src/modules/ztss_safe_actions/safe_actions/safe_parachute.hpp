#pragma once
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_event_trigger.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_servos.h>

#include <px4_platform_common/module_params.h>
#include "safe_action.hpp"

constexpr int PARACHUTE_SERVO_CHANNEL=8;

class ZtssActionParachute : public ModuleParams, public SafeActionBase
{
	public:
		ZtssActionParachute(ZtssSafeActions*, ModuleParams*);
		~ZtssActionParachute()=default;

		void start_execution() override;
		void end_execution() override;
		void execute_action() override;


		uint8_t get_action_id()const override;
		uint8_t get_execution_status()const override;

		void process_action_event() override;

	private:
		uORB::Publication<actuator_servos_s> servo_publisher_{ORB_ID(actuator_servos)};


};
