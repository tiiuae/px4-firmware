#pragma once
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_event_trigger.h>
#include <uORB/topics/ztss_safe_actions_status.h>
#include <px4_platform_common/module_params.h>
#include "helpers/actions.hpp"

class ZtssActionRTL : public ModuleParams, public SafeActionBase
{
	public:
		ZtssActionRTL(ModuleParams* parent);
		~ZtssActionRTL()=default;

	void start_execution();
	void end_execution();
	void execute_action();


	uint8_t get_action_id()const;
	uint8_t get_execution_status()const;

};
