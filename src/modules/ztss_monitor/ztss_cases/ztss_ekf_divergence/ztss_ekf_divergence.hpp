#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <limits>

#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

/**
 * @brief ZTSS Safety Case #13 – EKF divergence
 *
 * Monitors the EKF innovation test ratios (velocity, position, height, yaw/mag)
 * reported in estimator_status. When any ratio exceeds the configured limit the
 * filter is considered to have diverged; before arming this should deny arming,
 * after arming it should trigger the failsafe path.
 *
 * Thresholds are taken from the existing PX4 parameters COM_ARM_EKF_*.
 * Severity is derived from how far the worst-case ratio overshoots its limit
 * and includes a WARN band before CRITICAL.
 */

// Fraction of limit at which we enter the WARN band (e.g. 0.7 → warn when ratio > 70 % of limit)
static constexpr float EKF_WARN_FRACTION  = 0.7f;
// Fraction of limit at which we enter CRITICAL (ratio >= limit)
static constexpr float EKF_CRITICAL_FRACTION  = 1.0f;
// Margin percentages (0-100) used to decide LOW vs HIGH margin within each band
static constexpr float EKF_MARGIN_LOW_PCT = 0.75f;

// Hysteresis hold time before we declare the divergence confirmed
static constexpr hrt_abstime EKF_DIVERGENCE_HYSTERESIS_US = 2_s;


class ZtssCaseEkfDivergence : public ModuleParams
{
public:
	ZtssCaseEkfDivergence(ModuleParams *parent);
	~ZtssCaseEkfDivergence() = default;

	void update_subscribed_values();

	void construct_internal_business_data();

	void execute_use_case_safety_evaluation();

	void publish_use_case_status();

private:



	//  Publications
	uORB::Publication<ztss_monitor_use_case_output_s>
		ztss_use_case_ekf_divergence_pub_{ORB_ID(ztss_use_case_ekf_divergence)};

	//  Subscriptions
	uORB::Subscription estimator_status_sub_{ORB_ID(estimator_status)};

	//  Input messages
	estimator_status_s       estimator_status_input_{};
	estimator_status_flags_s estimator_status_flags_input_{};

	//  Output message
	ztss_monitor_use_case_output_s use_case_output_{};

	//  Internal state
	bool updated_subscriber_{false};

	// Per-channel hysteresis (velocity, position, height, yaw)
	systemlib::Hysteresis velocity_divergence_hysteresis_{false};
	systemlib::Hysteresis position_divergence_hysteresis_{false};
	systemlib::Hysteresis height_divergence_hysteresis_{false};
	systemlib::Hysteresis yaw_divergence_hysteresis_{false};

	// Aggregate divergence flag (any channel above limit after hysteresis)
	bool ekf_diverged_{false};

	// Computed per-cycle
	uint8_t severity_{ztss_monitor_use_case_output_s::INFO};
	uint8_t margin_{ztss_monitor_use_case_output_s::MARGIN_HIGH};

	// Normalised ratios (ratio / limit), kept for severity computation
	float velocity_normalized_{0.f};
	float position_normalized_{0.f};
	float height_normalized_{0.f};
	float yaw_normalized_{0.f};

	// Parameters (re-use the existing PX4 EKF arming-check limits)
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::COM_ARM_EKF_VEL>) _param_com_arm_ekf_vel,
		(ParamFloat<px4::params::COM_ARM_EKF_POS>) _param_com_arm_ekf_pos,
		(ParamFloat<px4::params::COM_ARM_EKF_HGT>) _param_com_arm_ekf_hgt,
		(ParamFloat<px4::params::COM_ARM_EKF_YAW>) _param_com_arm_ekf_yaw
	)
};
