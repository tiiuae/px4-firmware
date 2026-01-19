#include "ztss_case_dummy.hpp"

template<int NUMBER_OF_INPUTS>
ZtssCaseDummy<NUMBER_OF_INPUTS>::ZtssCaseDummy(ModuleParams *parent): ModuleParams(parent)
{
	// _ztss_use_case_dummy_pub.advertise();

	subscriptions_ = std::array<uORB::Subscription, NUMBER_OF_INPUTS>{
		uORB::Subscription{ORB_ID(ztss_dummy_trigger_a)},
		uORB::Subscription{ORB_ID(ztss_dummy_trigger_b)}
	};
}

template<int NUMBER_OF_INPUTS>
void ZtssCaseDummy<NUMBER_OF_INPUTS>::update_subscribed_values()
{
	for(size_t i =0; i<NUMBER_OF_INPUTS; i++)
	{
		if (subscriptions_[i].updated())
		{
			updated_subcribers_[i] = true;
			subscriptions_[i].copy(&messages_[i]);
			continue;
		}
		updated_subcribers_[i]=false;
	}
}

template<int NUMBER_OF_INPUTS>
void ZtssCaseDummy<NUMBER_OF_INPUTS>::execute_use_case_safety_evaluation()
{
	bool all_subcribers_updated = false;

	for(size_t i =0; i < NUMBER_OF_INPUTS; i++)
	{
		all_subcribers_updated = all_subcribers_updated && updated_subcribers_[i];
	}

	if (all_subcribers_updated)
	{
		bool use_case_valid_operation = false;
		for(size_t i =0; i < NUMBER_OF_INPUTS; i++)
		{
			use_case_valid_operation = use_case_valid_operation && messages_[i].request_type == ztss_monitor_use_case_output_s::VALID_EXECUTION;
		}

		this->use_case_output_.timestamp = hrt_absolute_time();
		this->use_case_output_.use_case_status = ztss_monitor_use_case_output_s::VALID_EXECUTION;


		if (!use_case_valid_operation)
		{
			this->use_case_output_.use_case_status = ztss_monitor_use_case_output_s::INVALID_EXECUTION;
		}
	}
}


template<int NUMBER_OF_INPUTS>
void ZtssCaseDummy<NUMBER_OF_INPUTS>::publish_use_case_status()
{
	this->_ztss_use_case_dummy_pub.publish(this->use_case_output_);
}


template class ZtssCaseDummy<2>;
