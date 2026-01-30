#include "safe_action_none.hpp"

ZtssActionNone::ZtssActionNone(ZtssSafeActions* parent, ModuleParams*module_params_parent):ModuleParams(module_params_parent), SafeActionBase(parent)
{
	this->action_id= ztss_event_trigger_s::ACTION_NONE;
};


void ZtssActionNone::start_execution()
{
	SafeActionBase::start_execution();
};

void ZtssActionNone::execute_action()
{
	SafeActionBase::execute_action();
};

void ZtssActionNone::end_execution(){

	SafeActionBase::end_execution();
	this->execution_status_ = ztss_safe_actions_status_s::NOT_EXECUTING;
};

void ZtssActionNone::process_action_event(){
	SafeActionBase::process_action_event();
}

uint8_t ZtssActionNone::get_action_id()const{return this->action_id;};

uint8_t ZtssActionNone::get_execution_status()const{return this->execution_status_;};
