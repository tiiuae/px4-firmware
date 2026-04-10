#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/satellite_info.h>
#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

/**
 * @brief ZTSS Safety Case #6 – GPS Failure
 *
 * Monitors GPS health using the sensor_gps, sensor_gnss_relative, and
 * satellite_info uORB topics published by the GPS driver.
 *
 * The sensor_gps topic provides fix type, satellite count, position accuracy
 * (eph/epv), dilution of precision (hdop/vdop), speed accuracy, and
 * jamming/spoofing indicators.
 *
 * The sensor_gnss_relative topic provides GNSS relative positioning data
 * including fix quality flags (gnss_fix_ok, differential_solution,
 * carrier_solution_fixed/floating) and position accuracy.
 *
 * The satellite_info topic provides per-satellite visibility, SNR, and
 * usage information.
 *
 * Detection checks:
 *   1. Fix type degraded           — fix_type < 3D (no 3D fix)
 *   2. Insufficient satellites     — satellites_used < EKF2_REQ_NSATS
 *   3. Position accuracy degraded  — eph > EKF2_REQ_EPH or epv > EKF2_REQ_EPV
 *   4. Speed accuracy degraded     — s_variance_m_s > EKF2_REQ_SACC
 *   5. PDOP exceeded               — pdop > EKF2_REQ_PDOP (computed from hdop/vdop)
 *   6. Jamming detected            — jamming_state >= WARNING
 *   7. Spoofing detected           — spoofing_state >= INDICATED
 *   8. GPS data stale              — topic not updated within timeout
 *   9. GNSS relative fix lost      — gnss_fix_ok = false (when relative used)
 *  10. Low satellite SNR           — average SNR below threshold
 *
 * Safe action: Land in attitude mode.
 */

// ── Thresholds ──────────────────────────────────────────────────────────────

/// GPS topic staleness timeout (microseconds): if no update in 3 seconds, GPS is stale
static constexpr hrt_abstime GPS_STALENESS_TIMEOUT_US = 3_s;

/// Hysteresis hold time before confirming GPS failure
static constexpr hrt_abstime GPS_FAILURE_HYSTERESIS_US = 1_s;

/// Minimum fix type required for healthy operation (3D fix)
static constexpr uint8_t GPS_MIN_FIX_TYPE_3D = 3;

/// Warn if fix type drops to 2D
static constexpr uint8_t GPS_WARN_FIX_TYPE_2D = 2;

/// Margin threshold within each severity band (0–1 range)
static constexpr float GPS_MARGIN_LOW_THRESHOLD = 0.75f;

/// Fraction of EPH threshold at which we enter WARN band (used when INFO)
static constexpr float GPS_EPH_WARN_FRACTION = 0.7f;


class ZtssCaseGpsFailure : public ModuleParams
{
public:
	/**
	 * @brief Constructor — initializes the GPS failure monitor case.
	 * @param parent Pointer to parent ModuleParams for parameter inheritance.
	 */
	ZtssCaseGpsFailure(ModuleParams *parent);
	~ZtssCaseGpsFailure() = default;

	/**
	 * @brief Polls subscribed uORB topics for new GPS data.
	 *
	 * Updates sensor_gps, sensor_gnss_relative, and satellite_info messages.
	 * Sets updated_subscribers_ flag if any new data was received.
	 */
	void update_subscribed_values();

	/**
	 * @brief Evaluates GPS health from fix quality, accuracy, satellites,
	 *        jamming/spoofing state, GNSS relative fix, and SNR.
	 *
	 * Computes per-check failure flags, aggregates into severity and margin,
	 * and feeds the hysteresis filter.
	 */
	void construct_internal_business_data();

	/**
	 * @brief Executes the safety evaluation based on hysteresis-debounced state.
	 *
	 * If GPS failure has persisted beyond the hysteresis period,
	 * marks the system as unhealthy with the computed severity and margin.
	 */
	void execute_use_case_safety_evaluation();

	/**
	 * @brief Publishes the safety evaluation result to uORB.
	 *
	 * Timestamps the output and publishes to ztss_use_case_gps_failure.
	 */
	void publish_use_case_status();

private:

	// ── Publications ────────────────────────────────────────────────────
	uORB::Publication<ztss_monitor_use_case_output_s>
		ztss_use_case_gps_failure_pub_{ORB_ID(ztss_use_case_gps_failure)};

	// ── Subscriptions ───────────────────────────────────────────────────
	uORB::Subscription sensor_gps_sub_{ORB_ID(sensor_gps)};
	uORB::Subscription sensor_gnss_relative_sub_{ORB_ID(sensor_gnss_relative)};
	uORB::Subscription satellite_info_sub_{ORB_ID(satellite_info)};

	// ── Input messages ──────────────────────────────────────────────────
	sensor_gps_s            sensor_gps_input_{};
	sensor_gnss_relative_s  sensor_gnss_relative_input_{};
	satellite_info_s        satellite_info_input_{};

	// ── Output message ──────────────────────────────────────────────────
	ztss_monitor_use_case_output_s use_case_output_{};

	// ── Internal state ──────────────────────────────────────────────────
	bool updated_subscribers_{false};
	bool gnss_relative_available_{false};  ///< True if we ever received a gnss_relative message

	// failure flags (computed each cycle)
	bool fix_type_failure_{false};         ///< Fix type below 3D
	bool satellite_count_failure_{false};  ///< Satellites below threshold
	bool eph_failure_{false};              ///< Horizontal accuracy exceeded
	bool epv_failure_{false};              ///< Vertical accuracy exceeded
	bool speed_accuracy_failure_{false};   ///< Speed accuracy exceeded
	bool pdop_failure_{false};             ///< PDOP exceeded
	bool jamming_detected_{false};         ///< Jamming state warning or critical
	bool spoofing_detected_{false};        ///< Spoofing state indicated or multiple
	bool gps_stale_{false};                ///< GPS topic data is stale
	bool gnss_relative_fix_lost_{false};   ///< GNSS relative fix lost

	// Aggregate GPS failure flag (fed into hysteresis)
	bool gps_failure_{false};

	// Hysteresis for GPS failure debouncing
	systemlib::Hysteresis gps_failure_hysteresis_{false};

	// Computed severity and margin
	uint8_t severity_{ztss_monitor_use_case_output_s::INFO};
	uint8_t margin_{ztss_monitor_use_case_output_s::MARGIN_HIGH};

	// ── Parameters (re-use existing EKF2 GPS quality thresholds against jamming) ────────
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::EKF2_REQ_EPH>)    _param_ekf2_req_eph,    ///< Max allowed horizontal position error (m)
		(ParamFloat<px4::params::EKF2_REQ_EPV>)    _param_ekf2_req_epv,    ///< Max allowed vertical position error (m)
		(ParamFloat<px4::params::EKF2_REQ_SACC>)   _param_ekf2_req_sacc,   ///< Max allowed speed accuracy (m/s)
		(ParamInt<px4::params::EKF2_REQ_NSATS>)    _param_ekf2_req_nsats,  ///< Min required satellite count
		(ParamFloat<px4::params::EKF2_REQ_PDOP>)   _param_ekf2_req_pdop    ///< Max allowed PDOP
	)
};
