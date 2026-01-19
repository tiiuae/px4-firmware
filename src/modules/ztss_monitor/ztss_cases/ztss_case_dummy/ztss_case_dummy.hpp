#pragma once
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/dataman_request.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <array>



class ZtssCaseDummy : public ModuleParams
{

public:

	ZtssCaseDummy(ModuleParams* parent);
	~ZtssCaseDummy() = default;

void update_subscribed_values();

void execute_use_case_safety_evaluation();

void publish_use_case_status();

private:
// TODO(renzo) make an array of subcriptions where each subscription has a idx.
// make a structure that take a number of templated subcription
uORB::Subscription _sitl_trigger_of_dummy_safe_actions_sub{ORB_ID(ztss_dummy_trigger)};
uORB::Publication<ztss_monitor_use_case_output_s> _ztss_use_case_dummy_pub{ORB_ID(ztss_use_case_dummy)};

private:
std::array<bool, 1> updated_subcribers_;

ztss_monitor_use_case_output_s use_case_output_;


};
