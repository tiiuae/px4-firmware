#include "safe_return_to_launch.hpp"


#include "safe_hold.hpp"


ZtssActionRTL::ZtssActionRTL(ModuleParams*parent):ModuleParams(parent)
{
	this->action_id= ztss_event_trigger_s::ACTION_RTL;
};


void ZtssActionRTL::start_execution(){};

void ZtssActionRTL::execute_action(){};

void ZtssActionRTL::end_execution(){};

uint8_t ZtssActionRTL::get_action_id()const{return this->action_id;};

uint8_t ZtssActionRTL::get_execution_status()const{return this->execution_status_;};



