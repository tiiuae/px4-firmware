#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/vehicle_attitude.h>
#include <limits>
#include <numbers>
#include <lib/hysteresis/hysteresis.h>


#include <px4_platform_common/module_params.h>


using namespace time_literals;

static const float WARN_THRESHOLD = M_PI / 6 ; // rad to max angle
static const float  CRITICAL_THRESHOLD = M_PI / 12; // rad

static const float MARGIN_LOW_WARN_THRESHOLD = 0.75f;
static const float MARGIN_LOW_CRITICAL_THRESHOLD =0.75f;


class ZtssCaseAttitudeFailure : public ModuleParams
{
public:
	/**
	 * @brief Constructor - initializes the attitude failure monitor case.
	 * @param parent Pointer to parent ModuleParams for parameter inheritance.
	 */
	ZtssCaseAttitudeFailure(ModuleParams* parent);

	/**
	 * @brief Destructor - cleans up resources (defaulted).
	 */
	~ZtssCaseAttitudeFailure()=default;

	/**
	 * @brief Polls subscribed uORB topics for new attitude data.
	 *
	 * Updates the `vehicle_attitude_input_` message from the `vehicle_attitude` topic
	 * if new data is available. Sets `updated_subscriber_` flag to track whether
	 * any data was received in this update cycle.
	 */
	void update_subscribed_values();

	/**
	 * @brief Computes internal business logic for attitude failure detection.
	 *
	 * Converts quaternion attitude to Euler angles (roll, pitch), compares against
	 * configured failure thresholds, computes severity level (INFO/WARN/CRITICAL),
	 * calculates safety margin, and updates hysteresis state machines for both
	 * roll and pitch failure modes.
	 */
	void construct_internal_bussiness_data();

	/**
	 * @brief Executes the safety evaluation logic.
	 *
	 * Checks hysteresis state of roll/pitch failures. If either axis has exceeded
	 * failure threshold (after debouncing), marks the system as unhealthy with
	 * appropriate severity and margin. If both axes are healthy, sets healthy=true
	 * with INFO severity and maximum margin.
	 */
	void execute_use_case_safety_evaluation();

	/**
	 * @brief Publishes the safety evaluation result to uORB.
	 *
	 * Timestamps the output message and publishes it to the
	 * `ztss_use_case_attitude_failure` topic for consumption by safety decision modules.
	 */
	void publish_use_case_status();


private:

// Publications
uORB::Publication<ztss_monitor_use_case_output_s> ztss_use_case_attitude_failure_pub_{ORB_ID(ztss_use_case_attitude_failure)};

// Subscriptions
uORB::Subscription vehicle_attitude_sub_{ORB_ID(vehicle_attitude)};


// messages output
ztss_monitor_use_case_output_s use_case_output_{};
// messages input
vehicle_attitude_s vehicle_attitude_input_{};

// internals

bool updated_subscriber_{false};

systemlib::Hysteresis roll_failure_hysteresis_{false};
systemlib::Hysteresis pitch_failure_hysteresis_{false};

uint8_t margin_to_next_severity_case{ztss_monitor_use_case_output_s::MARGIN_HIGH};
uint8_t severity{ztss_monitor_use_case_output_s::INFO};

DEFINE_PARAMETERS(
	(ParamInt<px4::params::FD_FAIL_P>) _param_fd_fail_p,
	(ParamInt<px4::params::FD_FAIL_R>) _param_fd_fail_r,
	(ParamFloat<px4::params::FD_FAIL_R_TTRI>) _param_fd_fail_r_ttri,
	(ParamFloat<px4::params::FD_FAIL_P_TTRI>) _param_fd_fail_p_ttri
)

};

