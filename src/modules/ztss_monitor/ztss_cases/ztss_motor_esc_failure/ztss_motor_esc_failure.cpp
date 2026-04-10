#include "ztss_motor_esc_failure.hpp"
#include <float.h>
#include <math.h>

ZtssCaseMotorEscFailure::ZtssCaseMotorEscFailure(ModuleParams *parent)
	: ModuleParams(parent)
{
	ztss_use_case_motor_esc_failure_pub_.advertise();
	esc_failure_hysteresis_.set_hysteresis_time_from(false, ESC_FAILURE_HYSTERESIS_US);
}

// ─────────────────────────────────────────────────────────────────────────────
// 1.  Read the latest ESC telemetry and vehicle status
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseMotorEscFailure::update_subscribed_values()
{
	bool any_updated = false;

	if (esc_status_sub_.updated()) {
		esc_status_sub_.copy(&esc_status_input_);
		any_updated = true;
	}



	updated_subscribers_ = any_updated;
}

// ─────────────────────────────────────────────────────────────────────────────
// 2.  Evaluate per-ESC health and compute aggregate severity
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseMotorEscFailure::construct_internal_business_data()
{
	const hrt_abstime time_now = hrt_absolute_time();

	// Reset per-cycle state
	esc_timed_out_mask_ = 0;
	esc_fault_mask_     = 0;
	failed_esc_count_   = 0;

	uint8_t worst_severity = ztss_monitor_use_case_output_s::INFO;
	uint8_t worst_margin   = ztss_monitor_use_case_output_s::MARGIN_HIGH;

	const int esc_count = esc_status_input_.esc_count;


	// --- Check each ESC
	for (int esc_index = 0; esc_index < esc_count; esc_index++) {
		const esc_report_s &report = esc_status_input_.esc[esc_index];

		// Map to actuator function index
		const unsigned motor_index = report.actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

		if (motor_index >= actuator_motors_s::NUM_CONTROLS) {
			continue;
		}

		bool this_esc_failed = false;
		uint8_t this_esc_severity = ztss_monitor_use_case_output_s::INFO;
		uint8_t this_esc_margin   = ztss_monitor_use_case_output_s::MARGIN_HIGH;

		// Track if this ESC has ever sent telemetry
		if (report.esc_current > 0.0f || report.timestamp > 0) {
			esc_had_telemetry_[motor_index] = true;
		}

		// --- Check 1: Telemetry timeout ---
		if (esc_had_telemetry_[motor_index]) {
			const hrt_abstime telemetry_age = time_now - report.timestamp;

			if (telemetry_age > ESC_TELEMETRY_TIMEOUT_US) {
				// ESC telemetry timed out — CRITICAL (ESC may be dead)
				esc_timed_out_mask_ |= (1u << motor_index);
				this_esc_failed = true;
				this_esc_severity = ztss_monitor_use_case_output_s::CRITICAL;
				this_esc_margin   = ztss_monitor_use_case_output_s::MARGIN_LOW;

			} else {
				// Compute margin: how close to timeout
				const float timeout_ratio = (float)telemetry_age / (float)ESC_TELEMETRY_TIMEOUT_US;

				if (timeout_ratio > ESC_MARGIN_LOW_THRESHOLD) {
					this_esc_margin = ztss_monitor_use_case_output_s::MARGIN_LOW;
				}
			}
		}

		// --- Check 2: ESC failure flags ---
		if (report.failures != 0) {
			esc_fault_mask_ |= (1u << motor_index);
			this_esc_failed = true;

			if (report.failures & ESC_CRITICAL_FAILURE_MASK) {
				this_esc_severity = ztss_monitor_use_case_output_s::CRITICAL;
				this_esc_margin   = ztss_monitor_use_case_output_s::MARGIN_LOW;

			} else if (report.failures & ESC_WARNING_FAILURE_MASK) {
				// Only upgrade to WARN if not already CRITICAL
				if (this_esc_severity < ztss_monitor_use_case_output_s::WARN) {
					this_esc_severity = ztss_monitor_use_case_output_s::WARN;
				}

				this_esc_margin = ztss_monitor_use_case_output_s::MARGIN_LOW;
			}
		}

		// --- Check 3: ESC offline (not in esc_online_flags bitmask) ---
		if (!(esc_status_input_.esc_online_flags & (1u << esc_index))) {
			this_esc_failed = true;

			if (this_esc_severity < ztss_monitor_use_case_output_s::WARN) {
				this_esc_severity = ztss_monitor_use_case_output_s::WARN;
			}

			this_esc_margin = ztss_monitor_use_case_output_s::MARGIN_LOW;
		}

		// --- Aggregate worst across all ESCs ---
		if (this_esc_failed) {
			failed_esc_count_++;
		}

		if (this_esc_severity > worst_severity) {
			worst_severity = this_esc_severity;
			worst_margin   = this_esc_margin;

		} else if (this_esc_severity == worst_severity
			   && this_esc_margin < worst_margin) {
			worst_margin = this_esc_margin;
		}
	}

	// Multiple ESC failures → escalate to CRITICAL regardless
	if (failed_esc_count_ > 1 && worst_severity < ztss_monitor_use_case_output_s::CRITICAL) {
		worst_severity = ztss_monitor_use_case_output_s::CRITICAL;
		worst_margin   = ztss_monitor_use_case_output_s::MARGIN_LOW;
	}

	any_esc_failure_ = (failed_esc_count_ > 0);
	severity_ = worst_severity;
	margin_   = worst_margin;

	// Feed the aggregate failure state into hysteresis
	esc_failure_hysteresis_.set_state_and_update(any_esc_failure_, time_now);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3.  Evaluate the safety case based on hysteresis state
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseMotorEscFailure::execute_use_case_safety_evaluation()
{
	if (!updated_subscribers_) {
		return;
	}

	if (esc_failure_hysteresis_.get_state()) {
		// At least one ESC has been in failure longer than the hysteresis period
		use_case_output_.healthy  = false;
		use_case_output_.severity = severity_;
		use_case_output_.margin   = margin_;

	} else {
		// All ESCs healthy (or transient glitch cleared within hysteresis window)
		use_case_output_.healthy  = true;
		use_case_output_.severity = ztss_monitor_use_case_output_s::INFO;
		use_case_output_.margin   = ztss_monitor_use_case_output_s::MARGIN_HIGH;
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// 4.  Publish the safety evaluation result
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseMotorEscFailure::publish_use_case_status()
{
	use_case_output_.timestamp = hrt_absolute_time();
	ztss_use_case_motor_esc_failure_pub_.publish(use_case_output_);
}
