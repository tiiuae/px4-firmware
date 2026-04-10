#include "ztss_vibration.hpp"

ZtssCaseVibration::ZtssCaseVibration(ModuleParams *parent)
	: ModuleParams(parent)
{
	ztss_use_case_vibration_pub_.advertise();

	// Configure duration-based hysteresis thresholds
	vibration_warn_hysteresis_.set_hysteresis_time_from(false, VIBRATION_WARN_HYSTERESIS_US);
	vibration_margin_low_hysteresis_.set_hysteresis_time_from(false, VIBRATION_MARGIN_LOW_HYSTERESIS_US);
	vibration_critical_hysteresis_.set_hysteresis_time_from(false, VIBRATION_CRITICAL_HYSTERESIS_US);
}

// ─────────────────────────────────────────────────────────────────────────────
// 1.  Read the latest IMU data
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseVibration::update_subscribed_values()
{
	if (vehicle_imu_sub_.updated()) {
		vehicle_imu_sub_.update(&vehicle_imu_input_);
		updated_subscriber_ = true;

	} else {
		updated_subscriber_ = false;
	}

	// Also poll sensors_status_imu (best-effort, not gating)
	if (sensors_status_imu_sub_.updated()) {
		sensors_status_imu_sub_.update(&sensors_status_imu_input_);
	}
}

// 2.  Compute vibration residual and feed hysteresis
void ZtssCaseVibration::construct_internal_business_data()
{
	if (!updated_subscriber_) {
		return;
	}

	const hrt_abstime time_now = hrt_absolute_time();

	//  Reconstruct raw acceleration (m/s²) from delta_velocity
	const float dt_s = static_cast<float>(vehicle_imu_input_.delta_velocity_dt) * 1e-6f;

	if (dt_s < 1e-6f) {
		// Invalid dt — skip this sample
		return;
	}

	const matrix::Vector3f raw_accel{
		vehicle_imu_input_.delta_velocity[0] / dt_s,
		vehicle_imu_input_.delta_velocity[1] / dt_s,
		vehicle_imu_input_.delta_velocity[2] / dt_s
	};

	// Initialise / configure the low-pass filter on first valid sample
	const float sample_freq = 1.0f / dt_s;

	if (!filter_initialised_) {
		accel_lpf_.set_cutoff_frequency(sample_freq, VIBRATION_LPF_CUTOFF_HZ);
		accel_lpf_.reset(raw_accel);
		filter_initialised_ = true;
		return; // first sample only seeds the filter
	}

	//  Apply low-pass filter
	const matrix::Vector3f filtered_accel = accel_lpf_.apply(raw_accel);

	//  Residual = raw − filtered (high-frequency vibration component) ─
	matrix::Vector3f residual = (raw_accel - filtered_accel).abs();


	//  Threshold comparison (each level has its own threshold)
	bool warn_exceeded_x = (residual(0) > VIBRATION_WARN_THRESHOLD_M_S2);       // > 20 m/s²
	bool margin_low_exceeded_x = (residual(0) > VIBRATION_MARGIN_LOW_THRESHOLD_M_S2 && warn_exceeded_x); // > 30 m/s²
	bool critical_exceeded_x   = (residual(0) > VIBRATION_CRITICAL_THRESHOLD_M_S2);   // > 35 m/s²


	//  Threshold comparison (each level has its own threshold)
	bool warn_exceeded_y = (residual(1) > VIBRATION_WARN_THRESHOLD_M_S2);       // > 20 m/s²
	bool margin_low_exceeded_y = (residual(1) > VIBRATION_MARGIN_LOW_THRESHOLD_M_S2 && warn_exceeded_y); // > 30 m/s²
	bool critical_exceeded_y   = (residual(1) > VIBRATION_CRITICAL_THRESHOLD_M_S2);   // > 35 m/s²

	//  Threshold comparison (each level has its own threshold)
	bool warn_exceeded_z = (residual(2) > VIBRATION_WARN_THRESHOLD_M_S2);       // > 20 m/s²
	bool margin_low_exceeded_z = (residual(2) > VIBRATION_MARGIN_LOW_THRESHOLD_M_S2 && warn_exceeded_z); // > 30 m/s²
	bool critical_exceeded_z   = (residual(2) > VIBRATION_CRITICAL_THRESHOLD_M_S2);   // > 35 m/s²



	// ── Duration-based hysteresis (each fed by its own threshold) ──────
	vibration_warn_hysteresis_.set_state_and_update(warn_exceeded_x || warn_exceeded_y || warn_exceeded_z, time_now);             // 20 m/s² for 300 ms
	vibration_margin_low_hysteresis_.set_state_and_update(margin_low_exceeded_x || margin_low_exceeded_y || margin_low_exceeded_z, time_now); // 30 m/s² for 2 s
	vibration_critical_hysteresis_.set_state_and_update(critical_exceeded_x || critical_exceeded_y || critical_exceeded_z, time_now);     // 35 m/s² for 1 s

	// ── Severity (duration-based) ───────────────────────────────────────
	if (vibration_critical_hysteresis_.get_state()) {
		severity_ = ztss_monitor_use_case_output_s::CRITICAL;

	} else if (vibration_warn_hysteresis_.get_state()) {
		severity_ = ztss_monitor_use_case_output_s::WARN;

	} else {
		severity_ = ztss_monitor_use_case_output_s::INFO;
	}

	// ── Margin ──────────────────────────────────────────────────────────
	if (vibration_margin_low_hysteresis_.get_state()) {
		margin_ = ztss_monitor_use_case_output_s::MARGIN_LOW;

	} else {
		margin_ = ztss_monitor_use_case_output_s::MARGIN_HIGH;
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// 3.  Evaluate the safety case based on hysteresis state
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseVibration::execute_use_case_safety_evaluation()
{
	if (updated_subscriber_) {
		if (severity_ > ztss_monitor_use_case_output_s::INFO) {
			use_case_output_.healthy = false;
			use_case_output_.severity = severity_;
			use_case_output_.margin = margin_;

		} else {
			use_case_output_.healthy = true;
			use_case_output_.severity = ztss_monitor_use_case_output_s::INFO;
			use_case_output_.margin = ztss_monitor_use_case_output_s::MARGIN_HIGH;
		}
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// 4.  Publish the safety evaluation result
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseVibration::publish_use_case_status()
{
	use_case_output_.timestamp = hrt_absolute_time();
	ztss_use_case_vibration_pub_.publish(use_case_output_);
}
