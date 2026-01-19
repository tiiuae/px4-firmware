#include "ztss_case_dummy.hpp"


ZtssCaseDummy::ZtssCaseDummy(ModuleParams *parent): ModuleParams(parent)
{
	_ztss_use_case_dummy_pub.advertise();
}


void ZtssCaseDummy::update_subscribed_values()
{
	// TODO (renzo) loop for the array of subcribers.
	if(_sitl_trigger_of_dummy_safe_actions_sub.updated())
	{
		PX4_INFO("New data on Dummy case input subcriber");
		this->updated_subcribers_[0] = true;

	}
	else
	{
		this->updated_subcribers_[0] = false;
	}
}

void ZtssCaseDummy::execute_use_case_safety_evaluation()
{

	this->use_case_output_.timestamp = hrt_absolute_time();
	this->use_case_output_.use_case_status = ztss_monitor_use_case_output_s::INVALID_EXECUTION;

}

void ZtssCaseDummy::publish_use_case_status()
{
	this->_ztss_use_case_dummy_pub.publish(this->use_case_output_);
}
