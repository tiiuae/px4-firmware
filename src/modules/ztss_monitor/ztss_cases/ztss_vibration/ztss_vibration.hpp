#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/sensors_status_imu.h>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

/**
 * @brief ZTSS Safety Case – Vibration Monitoring
 *
 * Monitors the vibration level of the drone by computing the high-frequency
 * residual of the accelerometer signal from the vehicle_imu topic.
 *
 * Algorithm:
 *   1. Read raw accelerometer data from vehicle_imu (delta_velocity / dt → m/s²)
 *   2. Apply a 2nd-order low-pass filter (Butterworth) with 12 Hz cutoff
 *   3. Compute residual = raw − filtered (high-frequency component)
 *   4. Compare the magnitude of the residual against per-level thresholds
 *
 * Severity (each level has its own threshold and duration):
 *   WARN     — residual > 20 m/s² sustained for > 300 ms
 *   CRITICAL — residual > 35 m/s² sustained for > 1 s
 *
 * Margin:
 *   MARGIN_LOW  — residual > 30 m/s² sustained for > 2 s
 *   MARGIN_HIGH — otherwise
 *
 * Input topics:
 *   - vehicle_imu       (raw IMU delta_velocity for accel reconstruction)
 *   - sensors_status_imu (accel health / consistency cross-check)
 */

/// Low-pass filter cutoff frequency (Hz)
static constexpr float VIBRATION_LPF_CUTOFF_HZ = 12.0f;

/// WARN vibration threshold (m/s²)
static constexpr float VIBRATION_WARN_THRESHOLD_M_S2     = 20.0f;

/// MARGIN_LOW vibration threshold (m/s²)
static constexpr float VIBRATION_MARGIN_LOW_THRESHOLD_M_S2 = 30.0f;

/// CRITICAL vibration threshold (m/s²)
static constexpr float VIBRATION_CRITICAL_THRESHOLD_M_S2 = 35.0f;

/// Hysteresis time for WARN detection (300 ms)
static constexpr hrt_abstime VIBRATION_WARN_HYSTERESIS_US = 300_ms;

/// Hysteresis time for MARGIN_LOW detection (2 s)
static constexpr hrt_abstime VIBRATION_MARGIN_LOW_HYSTERESIS_US = 2000_ms;

/// Hysteresis time for CRITICAL detection (1 s)
static constexpr hrt_abstime VIBRATION_CRITICAL_HYSTERESIS_US = 1000_ms;


class ZtssCaseVibration : public ModuleParams
{
public:
	/**
	 * @brief Constructor — initialises the vibration monitor case.
	 * @param parent Pointer to parent ModuleParams for parameter inheritance.
	 */
	ZtssCaseVibration(ModuleParams *parent);
	~ZtssCaseVibration() = default;

	/**
	 * @brief Polls subscribed uORB topics for new IMU data.
	 *
	 * Updates vehicle_imu and sensors_status_imu inputs.
	 * Sets updated_subscriber_ flag if new vehicle_imu data was received.
	 */
	void update_subscribed_values();

	/**
	 * @brief Computes vibration residual and feeds hysteresis.
	 *
	 * Reconstructs raw acceleration from delta_velocity / dt, applies
	 * a 12 Hz 2nd-order low-pass filter, computes the residual magnitude,
	 * and compares against per-level thresholds:
	 *   - WARN:       > 20 m/s² for 300 ms
	 *   - MARGIN_LOW: > 30 m/s² for 2 s
	 *   - CRITICAL:   > 35 m/s² for 1 s
	 */
	void construct_internal_business_data();

	/**
	 * @brief Executes the safety evaluation based on hysteresis-debounced state.
	 *
	 * Sets healthy/unhealthy and severity/margin from the duration-based
	 * hysteresis outputs.
	 */
	void execute_use_case_safety_evaluation();

	/**
	 * @brief Publishes the safety evaluation result to uORB.
	 *
	 * Timestamps the output and publishes to ztss_use_case_vibration.
	 */
	void publish_use_case_status();

private:

	// ── Publications ────────────────────────────────────────────────────
	uORB::Publication<ztss_monitor_use_case_output_s>
		ztss_use_case_vibration_pub_{ORB_ID(ztss_use_case_vibration)};

	// ── Subscriptions ───────────────────────────────────────────────────
	uORB::Subscription vehicle_imu_sub_{ORB_ID(vehicle_imu)};
	uORB::Subscription sensors_status_imu_sub_{ORB_ID(sensors_status_imu)};

	// ── Input messages ──────────────────────────────────────────────────
	vehicle_imu_s         vehicle_imu_input_{};
	sensors_status_imu_s  sensors_status_imu_input_{};

	// ── Output message ──────────────────────────────────────────────────
	ztss_monitor_use_case_output_s use_case_output_{};

	// ── Internal state ──────────────────────────────────────────────────
	bool updated_subscriber_{false};
	bool filter_initialised_{false};

	/// 2nd-order Butterworth low-pass filter for 3-axis acceleration
	math::LowPassFilter2p<matrix::Vector3f> accel_lpf_{};

	/// Duration-based hysteresis: WARN level (300 ms)
	systemlib::Hysteresis vibration_warn_hysteresis_{false};

	/// Duration-based hysteresis: MARGIN_LOW level (600 ms)
	systemlib::Hysteresis vibration_margin_low_hysteresis_{false};

	/// Duration-based hysteresis: CRITICAL level (1 s)
	systemlib::Hysteresis vibration_critical_hysteresis_{false};

	// Computed per-cycle
	uint8_t severity_{ztss_monitor_use_case_output_s::INFO};
	uint8_t margin_{ztss_monitor_use_case_output_s::MARGIN_HIGH};
};
