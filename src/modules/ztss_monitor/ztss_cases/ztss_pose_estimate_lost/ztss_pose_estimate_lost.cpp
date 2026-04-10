#include "ztss_pose_estimate_lost.hpp"
#include <float.h>
#include <math.h>

ZtssCasePoseEstimateLost::ZtssCasePoseEstimateLost(ModuleParams *parent)
	: ModuleParams(parent)
{
	ztss_use_case_pose_estimate_lost_pub_.advertise();
	position_lost_hysteresis_.set_hysteresis_time_from(false, POSE_LOST_HYSTERESIS_US);
}

// 1.  Read the latest position estimate and estimator status
void ZtssCasePoseEstimateLost::update_subscribed_values()
{
	bool any_updated = false;

	if (vehicle_local_position_sub_.updated()) {
		vehicle_local_position_sub_.copy(&vehicle_local_position_input_);
		any_updated = true;
	}

	if (estimator_status_sub_.updated()) {
		estimator_status_sub_.copy(&estimator_status_input_);
	}

	updated_subscribers_ = any_updated;
}

// 2.  Evaluate position estimate validity, accuracy, and staleness
void ZtssCasePoseEstimateLost::construct_internal_business_data()
{
	const hrt_abstime time_now = hrt_absolute_time();

	// Read failsafe thresholds from PX4 parameters
	const float eph_threshold     = _param_com_pos_fs_eph.get();     // metres
	const int staleness_limit_ms  = _param_com_pos_fs_delay.get();   // milliseconds
	const hrt_abstime staleness_limit_us = (hrt_abstime)staleness_limit_ms * 1000;

	// Individual checks

	// Check 1: Horizontal position validity flag
	const bool horizontal_position_invalid = !vehicle_local_position_input_.xy_valid;

	// Check 2: Vertical position validity flag
	const bool vertical_position_invalid = !vehicle_local_position_input_.z_valid;

	// Check 3: Heading not usable for control
	const bool heading_invalid = !vehicle_local_position_input_.heading_good_for_control;

	// Check 4: Horizontal position accuracy exceeds failsafe threshold
	const float current_eph = vehicle_local_position_input_.eph;
	const bool eph_exceeded = (eph_threshold > FLT_EPSILON) && (current_eph > eph_threshold);

	// Check 5: Topic timestamp staleness (estimator stopped publishing)
	const hrt_abstime position_age = time_now - vehicle_local_position_input_.timestamp;
	const bool position_stale = (staleness_limit_us > 0) && (position_age > staleness_limit_us);

	// Check 6: Dead-reckoning only (no absolute horizontal reference)
	const bool dead_reckoning_only = vehicle_local_position_input_.dead_reckoning;

	//  Severity computation
	// CRITICAL: horizontal position invalid, stale, or eph far exceeded
	// WARN:     vertical only invalid, heading invalid, dead-reckoning, or eph approaching limit
	// INFO:     all checks pass

	uint8_t computed_severity = ztss_monitor_use_case_output_s::INFO;
	uint8_t computed_margin   = ztss_monitor_use_case_output_s::MARGIN_HIGH;

	// Critical conditions: no usable horizontal position at all
	if (horizontal_position_invalid || position_stale || eph_exceeded) {
		computed_severity = ztss_monitor_use_case_output_s::CRITICAL;
		computed_margin   = ztss_monitor_use_case_output_s::MARGIN_LOW;

	} else if (vertical_position_invalid || heading_invalid || dead_reckoning_only) {
		// Warning conditions: degraded but partial position still available
		computed_severity = ztss_monitor_use_case_output_s::WARN;
		computed_margin   = ztss_monitor_use_case_output_s::MARGIN_LOW;

	} else {
		// All validity checks pass — compute margin from eph approaching threshold
		computed_severity = ztss_monitor_use_case_output_s::INFO;

		if (eph_threshold > FLT_EPSILON) {
			// How close eph is to the failsafe threshold (0.0 = perfect, 1.0 = at limit)
			const float eph_ratio = current_eph / eph_threshold;

			// Within INFO band, check if approaching WARN zone
			if (eph_ratio > POSE_EPH_WARN_FRACTION) {
				// Close to threshold — compute margin within the warn approach zone
				const float warn_band = 1.0f - POSE_EPH_WARN_FRACTION;
				const float progress = (warn_band > FLT_EPSILON)
						       ? ((eph_ratio - POSE_EPH_WARN_FRACTION) / warn_band)
						       : 1.0f;

				computed_margin = (progress > POSE_MARGIN_LOW_THRESHOLD)
						  ? ztss_monitor_use_case_output_s::MARGIN_LOW
						  : ztss_monitor_use_case_output_s::MARGIN_HIGH;

			} else {
				computed_margin = ztss_monitor_use_case_output_s::MARGIN_HIGH;
			}
		}
	}

	position_estimate_lost_ = (computed_severity >= ztss_monitor_use_case_output_s::WARN);
	severity_ = computed_severity;
	margin_   = computed_margin;

	// Feed the aggregate loss state into hysteresis
	position_lost_hysteresis_.set_state_and_update(position_estimate_lost_, time_now);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3.  Evaluate the safety case based on hysteresis state
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCasePoseEstimateLost::execute_use_case_safety_evaluation()
{
	if (!updated_subscribers_) {
		return;
	}

	if (position_lost_hysteresis_.get_state()) {
		// Position estimate has been lost/degraded beyond hysteresis period
		use_case_output_.healthy  = false;
		use_case_output_.severity = severity_;
		use_case_output_.margin   = margin_;

	} else {
		// Position estimate is valid (or transient loss cleared)
		use_case_output_.healthy  = true;
		use_case_output_.severity = severity_;
		use_case_output_.margin   = margin_;
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// 4.  Publish the safety evaluation result
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCasePoseEstimateLost::publish_use_case_status()
{
	use_case_output_.timestamp = hrt_absolute_time();
	ztss_use_case_pose_estimate_lost_pub_.publish(use_case_output_);
}
