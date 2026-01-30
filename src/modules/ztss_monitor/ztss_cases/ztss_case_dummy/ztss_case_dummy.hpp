#pragma once
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/dataman_request.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/ztss_monitor_use_case_output.h>


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
uORB::Publication<ztss_monitor_use_case_output_s> ztss_use_case_dummy_pub_{ORB_ID(ztss_use_case_dummy)};

private:
bool updated_subcriber_{};
dataman_request_s input_message_{};
ztss_monitor_use_case_output_s use_case_output_{};

uORB::Subscription ztss_dummy_case_subscription_{ORB_ID(ztss_dummy_trigger)};

};
