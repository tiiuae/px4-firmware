#include "ztss_imu_failure.hpp"

ZtssCaseImuFailure::ZtssCaseImuFailure(ModuleParams *parent)
	: ModuleParams(parent)
{
	ztss_use_case_imu_failure_pub_.advertise();

	const hrt_abstime hysteresis_us = (hrt_abstime)(1_s * IMU_FAILURE_HYSTERESIS_TIME_S);

	accel_failure_hysteresis_.set_hysteresis_time_from(false, hysteresis_us);
	gyro_failure_hysteresis_.set_hysteresis_time_from(false, hysteresis_us);
}

void ZtssCaseImuFailure::update_subscribed_values()
{
	bool any_updated = false;

	if (sensors_status_imu_sub_.updated()) {
		sensors_status_imu_sub_.update(&sensors_status_imu_input_);
		any_updated = true;
	}

	updated_subscribers_ = any_updated;
}

bool ZtssCaseImuFailure::evaluate_sensor_type(
	const bool healthy[IMU_MAX_SENSOR_INSTANCES],
	const float inconsistency[IMU_MAX_SENSOR_INSTANCES],
	const uint32_t device_ids[IMU_MAX_SENSOR_INSTANCES],
	uint32_t primary_id,
	float warn_threshold,
	float critical_threshold,
	uint8_t &sensor_severity,
	uint8_t &sensor_margin)
{
	bool has_failure = false;
	bool primary_unhealthy = false;
	uint8_t populated_count = 0;
	uint8_t unhealthy_count = 0;
	float max_inconsistency = 0.0f;

	for (uint8_t i = 0; i < IMU_MAX_SENSOR_INSTANCES; i++) {
		// Skip unpopulated sensor slots (device_id == 0)
		if (device_ids[i] == 0) {
			continue;
		}

		populated_count++;

		if (!healthy[i]) {
			unhealthy_count++;
			has_failure = true;

			// Check if the primary sensor is the one failing
			if (device_ids[i] == primary_id) {
				primary_unhealthy = true;
			}
		}

		if (inconsistency[i] > max_inconsistency) {
			max_inconsistency = inconsistency[i];
		}
	}

	// Severity:
	// - CRITICAL: primary sensor unhealthy, all populated sensors unhealthy,
	//             or inconsistency exceeds critical threshold
	// - WARN:     any non-primary sensor unhealthy, or inconsistency exceeds warn threshold
	// - INFO:     all sensors healthy and consistent
	if (primary_unhealthy || (populated_count > 0 && unhealthy_count == populated_count) || max_inconsistency > critical_threshold) {
		sensor_severity = ztss_monitor_use_case_output_s::CRITICAL;

	} else if (unhealthy_count > 0 || max_inconsistency > warn_threshold) {
		sensor_severity = ztss_monitor_use_case_output_s::WARN;

	} else {
		sensor_severity = ztss_monitor_use_case_output_s::INFO;
	}

	// MARGIN
	switch (sensor_severity) {
	case ztss_monitor_use_case_output_s::INFO: {
		// Margin towards WARN: how close inconsistency is to warn threshold
		float ratio = (warn_threshold > 0.0f) ? (max_inconsistency / warn_threshold) : 0.0f;

		sensor_margin = (ratio > IMU_MARGIN_LOW_THRESHOLD)
				? ztss_monitor_use_case_output_s::MARGIN_LOW
				: ztss_monitor_use_case_output_s::MARGIN_HIGH;
		break;
	}

	case ztss_monitor_use_case_output_s::WARN: {
		// Margin towards CRITICAL: how close inconsistency is to critical threshold
		float range = critical_threshold - warn_threshold;
		float ratio = (range > 0.0f)
			      ? ((max_inconsistency - warn_threshold) / range)
			      : 1.0f;
		sensor_margin = (ratio > IMU_MARGIN_LOW_THRESHOLD)
				? ztss_monitor_use_case_output_s::MARGIN_LOW
				: ztss_monitor_use_case_output_s::MARGIN_HIGH;
		break;
	}

	default:
		// CRITICAL: already at worst severity
		sensor_margin = ztss_monitor_use_case_output_s::MARGIN_LOW;
		break;
	}

	return has_failure;
}

void ZtssCaseImuFailure::construct_internal_business_data()
{
	// --- Evaluate accel sensors ---
	uint8_t accel_severity = ztss_monitor_use_case_output_s::INFO;
	uint8_t accel_margin   = ztss_monitor_use_case_output_s::MARGIN_HIGH;
	accel_failure_ = evaluate_sensor_type(
				 sensors_status_imu_input_.accel_healthy,
				 sensors_status_imu_input_.accel_inconsistency_m_s_s,
				 sensors_status_imu_input_.accel_device_ids,
				 sensors_status_imu_input_.accel_device_id_primary,
				 ACCEL_INCONSISTENCY_WARN,
				 ACCEL_INCONSISTENCY_CRITICAL,
				 accel_severity,
				 accel_margin);

	// --- Evaluate gyro sensors ---
	uint8_t gyro_severity = ztss_monitor_use_case_output_s::INFO;
	uint8_t gyro_margin   = ztss_monitor_use_case_output_s::MARGIN_HIGH;
	gyro_failure_ = evaluate_sensor_type(
				sensors_status_imu_input_.gyro_healthy,
				sensors_status_imu_input_.gyro_inconsistency_rad_s,
				sensors_status_imu_input_.gyro_device_ids,
				sensors_status_imu_input_.gyro_device_id_primary,
				GYRO_INCONSISTENCY_WARN,
				GYRO_INCONSISTENCY_CRITICAL,
				gyro_severity,
				gyro_margin);

	// --- Aggregate: take the worst severity across accel and gyro ---
	uint8_t severities[] = { accel_severity, gyro_severity };
	uint8_t margins[]    = { accel_margin,   gyro_margin   };

	severity_ = ztss_monitor_use_case_output_s::INFO;
	margin_   = ztss_monitor_use_case_output_s::MARGIN_HIGH;

	for (uint8_t i = 0; i < 2; i++) {
		if (severities[i] > severity_) {
			severity_ = severities[i];
			margin_   = margins[i];

		} else if (severities[i] == severity_ && margins[i] < margin_) {
			// Same severity but tighter margin (MARGIN_LOW < MARGIN_HIGH)
			margin_ = margins[i];
		}
	}



	// --- Update hysteresis for each sensor type ---
	const hrt_abstime time_now = hrt_absolute_time();

	accel_failure_hysteresis_.set_state_and_update(accel_failure_, time_now);
	gyro_failure_hysteresis_.set_state_and_update(gyro_failure_, time_now);
}

void ZtssCaseImuFailure::execute_use_case_safety_evaluation()
{
	if (!updated_subscribers_) {
		return;
	}

	const bool any_sensor_failed = accel_failure_hysteresis_.get_state()
				       || gyro_failure_hysteresis_.get_state();

	if (any_sensor_failed) {
		use_case_output_.healthy  = false;
		use_case_output_.severity = severity_;
		use_case_output_.margin   = margin_;

	} else {
		use_case_output_.healthy  = true;
		use_case_output_.severity = ztss_monitor_use_case_output_s::INFO;
		use_case_output_.margin   = ztss_monitor_use_case_output_s::MARGIN_HIGH;
	}
}

void ZtssCaseImuFailure::publish_use_case_status()
{
	use_case_output_.timestamp = hrt_absolute_time();
	ztss_use_case_imu_failure_pub_.publish(use_case_output_);
}
