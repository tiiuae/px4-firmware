#include "ztss_velocity_estimate_lost.hpp"

ZtssCaseVelocityEstimateLost::ZtssCaseVelocityEstimateLost(ModuleParams *parent)
	: ModuleParams(parent)
{
	ztss_use_case_velocity_estimate_lost_pub_.advertise();
}

void ZtssCaseVelocityEstimateLost::update_subscribed_values()
{
	// TODO: copy from relevant uORB subscriptions
	updated_subscribers_ = false;
}

void ZtssCaseVelocityEstimateLost::execute_use_case_safety_evaluation()
{
	// TODO: implement velocity-estimate-lost safety evaluation
}

void ZtssCaseVelocityEstimateLost::publish_use_case_status()
{
	use_case_output_.timestamp = hrt_absolute_time();
	ztss_use_case_velocity_estimate_lost_pub_.publish(use_case_output_);
}
