#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/sensors_status.h>
#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

// Maximum number of sensor instances per type (PX4 supports up to 4)
static constexpr uint8_t BARO_MAX_SENSOR_INSTANCES = 4;

// Inconsistency thresholds for baro (Pa)
static constexpr float BARO_INCONSISTENCY_WARN      = 100.0f;
static constexpr float BARO_INCONSISTENCY_CRITICAL   = 200.0f;

// Margin ratio threshold (0–1 range: 0.75 = 75% towards next severity)
static constexpr float BARO_MARGIN_LOW_THRESHOLD = 0.75f;

// Hysteresis time for sensor failure debouncing (seconds)
static constexpr float BARO_FAILURE_HYSTERESIS_TIME_S = 0.3f;


class ZtssCaseBarometerFailure : public ModuleParams
{
public:
	ZtssCaseBarometerFailure(ModuleParams *parent);
	~ZtssCaseBarometerFailure() = default;

	void update_subscribed_values();

	void construct_internal_business_data();

	void execute_use_case_safety_evaluation();

	void publish_use_case_status();

private:

	/**
	 * @brief Evaluate barometer sensors.
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
	bool evaluate_sensor_type(const bool healthy[BARO_MAX_SENSOR_INSTANCES],
				  const float inconsistency[BARO_MAX_SENSOR_INSTANCES],
				  const uint32_t device_ids[BARO_MAX_SENSOR_INSTANCES],
				  uint32_t primary_id,
				  float warn_thresh,
				  float critical_thresh,
				  uint8_t &sensor_severity,
				  uint8_t &sensor_margin);

	// Publications
	uORB::Publication<ztss_monitor_use_case_output_s> ztss_use_case_barometer_failure_pub_{ORB_ID(ztss_use_case_barometer_failure)};

	// Subscriptions
	uORB::Subscription sensors_status_baro_sub_{ORB_ID(sensors_status_baro)};

	// Messages input
	sensors_status_s sensors_status_baro_input_{};

	// Messages output
	ztss_monitor_use_case_output_s use_case_output_{};

	// Internals
	bool updated_subscribers_{false};

	// Failure status (fed into hysteresis)
	bool baro_failure_{false};

	// Hysteresis for debouncing
	systemlib::Hysteresis baro_failure_hysteresis_{false};

	// Aggregated severity and margin
	uint8_t severity_{ztss_monitor_use_case_output_s::INFO};
	uint8_t margin_{ztss_monitor_use_case_output_s::MARGIN_HIGH};
};
