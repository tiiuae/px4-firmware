#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <px4_platform_common/module_params.h>


class ZtssCaseVelocityEstimateLost : public ModuleParams
{
public:
	ZtssCaseVelocityEstimateLost(ModuleParams *parent);
	~ZtssCaseVelocityEstimateLost() = default;

	void update_subscribed_values();

	void execute_use_case_safety_evaluation();

	void publish_use_case_status();

private:
	// Publications
	uORB::Publication<ztss_monitor_use_case_output_s> ztss_use_case_velocity_estimate_lost_pub_{ORB_ID(ztss_use_case_velocity_estimate_lost)};

	// Subscriptions

	// Messages
	ztss_monitor_use_case_output_s use_case_output_{};

	// Internals
	bool updated_subscribers_{false};
};
