#pragma once
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_event_trigger.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/ztss_safe_actions_status.h>
#include <px4_platform_common/module_params.h>
#include "helpers/actions.hpp"
#include "safe_action.hpp"

class ZtssActionNone : public ModuleParams, public SafeActionBase
{

	public:
	ZtssActionNone(ZtssSafeActions*, ModuleParams*);
	~ZtssActionNone()=default;

	void start_execution() override;
	void end_execution() override;
	void execute_action() override;


	uint8_t get_action_id() const override;
	uint8_t get_execution_status() const override;


	void process_action_event() override;

};


