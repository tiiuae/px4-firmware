#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/sensors_status.h>
#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

// Maximum number of sensor instances per type (PX4 supports up to 4)
static constexpr uint8_t MAG_MAX_SENSOR_INSTANCES = 4;

// Inconsistency thresholds for mag (gauss)
static constexpr float MAG_INCONSISTENCY_WARN       = 0.3f;
static constexpr float MAG_INCONSISTENCY_CRITICAL    = 0.5f;

// Margin ratio threshold (0–1 range: 0.75 = 75% towards next severity)
static constexpr float MAG_MARGIN_LOW_THRESHOLD = 0.75f;

// Hysteresis time for sensor failure debouncing (seconds)
static constexpr float MAG_FAILURE_HYSTERESIS_TIME_S = 0.3f;


class ZtssCaseMagnetometerFailure : public ModuleParams
{
public:
	ZtssCaseMagnetometerFailure(ModuleParams *parent);
	~ZtssCaseMagnetometerFailure() = default;

	void update_subscribed_values();

	void construct_internal_business_data();

	void execute_use_case_safety_evaluation();

	void publish_use_case_status();

private:

	/**
	 * @brief Evaluate magnetometer sensors.
	 * @param healthy         Array of per-instance healthy flags
	 * @param inconsistency   Array of per-instance inconsistency values
	 * @param device_ids      Array of per-instance device IDs (0 = unpopulated)
	 * @param primary_id      Device ID of the currently selected primary sensor
	 * @param warn_thresh     Inconsistency threshold for WARN
	 * @param critical_thresh Inconsistency threshold for CRITICAL
	 * @param[out] sensor_severity  Computed severity for this sensor type
	 * @param[out] sensor_margin    Computed margin for this sensor type
	 * @return true if any sensor of this type is in failure
	 */
	bool evaluate_sensor_type(const bool healthy[MAG_MAX_SENSOR_INSTANCES],
				  const float inconsistency[MAG_MAX_SENSOR_INSTANCES],
				  const uint32_t device_ids[MAG_MAX_SENSOR_INSTANCES],
				  uint32_t primary_id,
				  float warn_thresh,
				  float critical_thresh,
				  uint8_t &sensor_severity,
				  uint8_t &sensor_margin);

	// Publications
	uORB::Publication<ztss_monitor_use_case_output_s> ztss_use_case_magnetometer_failure_pub_{ORB_ID(ztss_use_case_magnetometer_failure)};

	// Subscriptions
	uORB::Subscription sensors_status_mag_sub_{ORB_ID(sensors_status_mag)};

	// Messages input
	sensors_status_s sensors_status_mag_input_{};

	// Messages output
	ztss_monitor_use_case_output_s use_case_output_{};

	// Internals
	bool updated_subscribers_{false};

	// Failure status (fed into hysteresis)
	bool mag_failure_{false};

	// Hysteresis for debouncing
	systemlib::Hysteresis mag_failure_hysteresis_{false};

	// Aggregated severity and margin
	uint8_t severity_{ztss_monitor_use_case_output_s::INFO};
	uint8_t margin_{ztss_monitor_use_case_output_s::MARGIN_HIGH};
};
