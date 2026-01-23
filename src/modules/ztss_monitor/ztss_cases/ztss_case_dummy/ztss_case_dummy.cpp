#include "ztss_case_dummy.hpp"

ZtssCaseDummy::ZtssCaseDummy(ModuleParams *parent): ModuleParams(parent)
{
	ztss_use_case_dummy_pub_.advertise();

}

void ZtssCaseDummy::update_subscribed_values()
{


		if (ztss_dummy_case_subscription_.updated())
		{
			updated_subcriber_ = true;
			ztss_dummy_case_subscription_.copy(&input_message_);
			PX4_INFO("Received a dummy tirgger %d", static_cast<int>(this->input_message_.request_type));
			return;
		}
		updated_subcriber_=false;
}

void ZtssCaseDummy::execute_use_case_safety_evaluation()
{


	if (updated_subcriber_)
	{

		if (this->input_message_.request_type == ztss_monitor_use_case_output_s::INFO)
		{
			this->use_case_output_.healthy = true;
			this->use_case_output_.severity = ztss_monitor_use_case_output_s::INFO;
			this->use_case_output_.margin = ztss_monitor_use_case_output_s::MARGIN_LOW;
		}
		if (this->input_message_.request_type == ztss_monitor_use_case_output_s::WARN)
		{
			this->use_case_output_.healthy = false;
			this->use_case_output_.severity = ztss_monitor_use_case_output_s::WARN;
			this->use_case_output_.margin = ztss_monitor_use_case_output_s::MARGIN_LOW;
		}
		if (this->input_message_.request_type == ztss_monitor_use_case_output_s::CRITICAL)
		{
			this->use_case_output_.healthy = false;
			this->use_case_output_.severity = ztss_monitor_use_case_output_s::CRITICAL;
			this->use_case_output_.margin = ztss_monitor_use_case_output_s::MARGIN_LOW;
		}
	}
}


void ZtssCaseDummy::publish_use_case_status()
{
	this->use_case_output_.timestamp= hrt_absolute_time();
	this->ztss_use_case_dummy_pub_.publish(this->use_case_output_);
}


