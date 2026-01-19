#pragma once
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/dataman_request.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <array>
#include <any>


template<int NUMBER_OF_INPUTS>
class ZtssCaseDummy : public ModuleParams
{

public:

	ZtssCaseDummy(ModuleParams* parent);
	~ZtssCaseDummy() = default;

void update_subscribed_values();

void execute_use_case_safety_evaluation();

void publish_use_case_status();

private:

// Publications
uORB::Publication<ztss_monitor_use_case_output_s> _ztss_use_case_dummy_pub{ORB_ID(ztss_use_case_dummy)};

private:
std::array<bool, NUMBER_OF_INPUTS> updated_subcribers_;
std::array<uORB::Subscription, NUMBER_OF_INPUTS> subscriptions_;
std::array<dataman_request_s, NUMBER_OF_INPUTS> messages_;
ztss_monitor_use_case_output_s use_case_output_;

};
