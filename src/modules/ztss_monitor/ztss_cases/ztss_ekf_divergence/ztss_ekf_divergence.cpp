#include "ztss_ekf_divergence.hpp"
#include <float.h>
#include <math.h>

ZtssCaseEkfDivergence::ZtssCaseEkfDivergence(ModuleParams *parent)
	: ModuleParams(parent)
{
	ztss_use_case_ekf_divergence_pub_.advertise();

	// Initialise hysteresis timers
	velocity_divergence_hysteresis_.set_hysteresis_time_from(false, EKF_DIVERGENCE_HYSTERESIS_US);
	position_divergence_hysteresis_.set_hysteresis_time_from(false, EKF_DIVERGENCE_HYSTERESIS_US);
	height_divergence_hysteresis_.set_hysteresis_time_from(false, EKF_DIVERGENCE_HYSTERESIS_US);
	yaw_divergence_hysteresis_.set_hysteresis_time_from(false, EKF_DIVERGENCE_HYSTERESIS_US);
}

// 1.  Read the latest estimator_status from EKF2
void ZtssCaseEkfDivergence::update_subscribed_values()
{
	bool got_update = false;

	if (estimator_status_sub_.updated()) {
		estimator_status_sub_.copy(&estimator_status_input_);
		got_update = true;

	}

	updated_subscriber_ = got_update;
}

// 2.  Normalise the innovation test-ratios and feed the hysteresis filters
void ZtssCaseEkfDivergence::construct_internal_business_data()
{
	// Read param limits (may change at runtime)
	const float velocity_limit = _param_com_arm_ekf_vel.get();
	const float position_limit = _param_com_arm_ekf_pos.get();
	const float height_limit = _param_com_arm_ekf_hgt.get();
	const float yaw_limit = _param_com_arm_ekf_yaw.get();

	// Normalise each ratio to its limit (0 = perfect, 1 = at limit)
	velocity_normalized_ = (velocity_limit > FLT_EPSILON) ? (estimator_status_input_.vel_test_ratio / velocity_limit) : 0.f;
	position_normalized_ = (position_limit > FLT_EPSILON) ? (estimator_status_input_.pos_test_ratio / position_limit) : 0.f;
	height_normalized_ = (height_limit > FLT_EPSILON) ? (estimator_status_input_.hgt_test_ratio / height_limit) : 0.f;
	yaw_normalized_ = (yaw_limit > FLT_EPSILON) ? (estimator_status_input_.mag_test_ratio / yaw_limit) : 0.f;


	// Boolean: is each channel above its limit right now?
	const bool velocity_over = velocity_normalized_ >= EKF_CRITICAL_FRACTION;
	const bool position_over = position_normalized_ >= EKF_CRITICAL_FRACTION;
	const bool height_over = height_normalized_ >= EKF_CRITICAL_FRACTION;
	const bool yaw_over = yaw_normalized_ >= EKF_CRITICAL_FRACTION;

	// Feed hysteresis filters
	const hrt_abstime now = hrt_absolute_time();
	velocity_divergence_hysteresis_.set_state_and_update(velocity_over, now);
	position_divergence_hysteresis_.set_state_and_update(position_over, now);
	height_divergence_hysteresis_.set_state_and_update(height_over, now);
	yaw_divergence_hysteresis_.set_state_and_update(yaw_over, now);

	// Aggregate: any channel confirmed diverged
	ekf_diverged_ = velocity_divergence_hysteresis_.get_state()
			|| position_divergence_hysteresis_.get_state()
			|| height_divergence_hysteresis_.get_state()
			|| yaw_divergence_hysteresis_.get_state();

	// Compute severity / margin from worst channel
	float worst_ratio = velocity_normalized_;
	if (position_normalized_ > worst_ratio) { worst_ratio = position_normalized_; }
	if (height_normalized_ > worst_ratio) { worst_ratio = height_normalized_; }
	if (yaw_normalized_ > worst_ratio) { worst_ratio = yaw_normalized_; }


	if (worst_ratio >= EKF_CRITICAL_FRACTION) {
		severity_ = ztss_monitor_use_case_output_s::CRITICAL;

	} else if (worst_ratio >= EKF_WARN_FRACTION) {
		severity_ = ztss_monitor_use_case_output_s::WARN;

	} else {
		severity_ = ztss_monitor_use_case_output_s::INFO;
	}

	// Margin to next severity level
	// Within each band we express how far (%) we have progressed towards
	// the next boundary. Above EKF_MARGIN_LOW_PCT ⇒ MARGIN_LOW.
	switch (severity_) {
	case ztss_monitor_use_case_output_s::INFO: {
		// Band is [0, WARN). Progress = ratio / WARN_FRACTION
		const float progress = (EKF_WARN_FRACTION > FLT_EPSILON)
				       ? (worst_ratio / EKF_WARN_FRACTION) : 0.f;
		margin_ = (progress > EKF_MARGIN_LOW_PCT)
			  ? ztss_monitor_use_case_output_s::MARGIN_LOW
			  : ztss_monitor_use_case_output_s::MARGIN_HIGH;
		break;
	}

	case ztss_monitor_use_case_output_s::WARN: {
		// Band is [WARN, CRIT). Progress = (ratio - WARN) / (CRIT - WARN)
		const float band = EKF_CRITICAL_FRACTION - EKF_WARN_FRACTION;
		const float progress = (band > FLT_EPSILON)
				       ? ((worst_ratio - EKF_WARN_FRACTION) / band) : 1.f;
		margin_ = (progress > EKF_MARGIN_LOW_PCT)
			  ? ztss_monitor_use_case_output_s::MARGIN_LOW
			  : ztss_monitor_use_case_output_s::MARGIN_HIGH;
		break;
	}

	default:
		// CRITICAL – no further band, margin is always LOW
		margin_ = ztss_monitor_use_case_output_s::MARGIN_LOW;
		break;
	}}

// 3.  Evaluate the safety case
void ZtssCaseEkfDivergence::execute_use_case_safety_evaluation()
{
	if (!updated_subscriber_) {
		return;
	}

	if (ekf_diverged_) {
		// At least one channel has been over its limit for the hysteresis period
		use_case_output_.healthy  = false;
		use_case_output_.severity = severity_;
		use_case_output_.margin   = margin_;

	} else {
		// All channels within limits (or transient spike already cleared)
		use_case_output_.healthy  = true;
		use_case_output_.severity = severity_;
		use_case_output_.margin   = margin_;
	}
}

// 4.  Publish
void ZtssCaseEkfDivergence::publish_use_case_status()
{
	use_case_output_.timestamp = hrt_absolute_time();
	ztss_use_case_ekf_divergence_pub_.publish(use_case_output_);
}

