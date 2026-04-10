#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

/**
 * @brief ZTSS Safety Case #3 – Motor / ESC failure
 *
 * Monitors the Electronic Speed Controllers (ESCs) via the esc_status uORB
 * topic, which is published by ESC drivers (DShot, UAVCAN/DroneCAN, TAP ESC,
 * VOXL ESC, etc.) and simulator bridges.
 *
 * The esc_status topic contains an array of 8 esc_report structs, each
 * reporting per-ESC telemetry: RPM, current, voltage, temperature, error count,
 * and a failure bitmask (overcurrent, overvoltage, overtemp, stuck, etc.).
 *
 * Detection checks (only while armed):
 *   1. Telemetry timeout — ESC stops reporting for > TELEMETRY_TIMEOUT_US
 *   2. ESC failure flags — the esc_report.failures bitmask is non-zero
 *   3. Offline ESC      — esc_online_flags bitmask shows ESC missing
 *
 * Severity:
 *   CRITICAL — primary motor ESC failure, or multiple ESC failures
 *   WARN     — single non-primary ESC failure or approaching timeout
 *   INFO     — all ESCs healthy
 */

// Telemetry timeout threshold (same as FailureDetector: 300ms)
static constexpr hrt_abstime ESC_TELEMETRY_TIMEOUT_US = 300_ms;

// Hysteresis time before confirming ESC failure (debounce transient glitches)
static constexpr hrt_abstime ESC_FAILURE_HYSTERESIS_US = 500_ms;

// Margin threshold (0–1 range): fraction of timeout elapsed → MARGIN_LOW
static constexpr float ESC_MARGIN_LOW_THRESHOLD = 0.75f;

// Critical ESC failure bitmask bits (hard faults)
static constexpr uint16_t ESC_CRITICAL_FAILURE_MASK =
	(1u << esc_report_s::FAILURE_OVER_CURRENT)
	| (1u << esc_report_s::FAILURE_OVER_VOLTAGE)
	| (1u << esc_report_s::FAILURE_MOTOR_OVER_TEMPERATURE)
	| (1u << esc_report_s::FAILURE_MOTOR_STUCK)
	| (1u << esc_report_s::FAILURE_OVER_ESC_TEMPERATURE);

// Warning ESC failure bitmask bits (soft faults)
static constexpr uint16_t ESC_WARNING_FAILURE_MASK =
	(1u << esc_report_s::FAILURE_OVER_RPM)
	| (1u << esc_report_s::FAILURE_INCONSISTENT_CMD)
	| (1u << esc_report_s::FAILURE_GENERIC)
	| (1u << esc_report_s::FAILURE_MOTOR_WARN_TEMPERATURE)
	| (1u << esc_report_s::FAILURE_WARN_ESC_TEMPERATURE);


class ZtssCaseMotorEscFailure : public ModuleParams
{
public:
	/**
	 * @brief Constructor — initializes the motor/ESC failure monitor case.
	 * @param parent Pointer to parent ModuleParams for parameter inheritance.
	 */
	ZtssCaseMotorEscFailure(ModuleParams *parent);
	~ZtssCaseMotorEscFailure() = default;

	/**
	 * @brief Polls subscribed uORB topics for new ESC telemetry data.
	 *
	 * Updates esc_status, vehicle_status from their respective topics.
	 * Sets updated_subscribers_ flag if any new data was received.
	 */
	void update_subscribed_values();

	/**
	 * @brief Evaluates per-ESC health from telemetry data.
	 *
	 * For each ESC: checks telemetry age (timeout), failure bitmask flags,
	 * and online status. Computes aggregate severity (CRITICAL/WARN/INFO)
	 * and margin. Feeds per-ESC failure states into the hysteresis filter.
	 */
	void construct_internal_business_data();

	/**
	 * @brief Executes the safety evaluation based on hysteresis-debounced state.
	 *
	 * If any ESC has been in failure state longer than the hysteresis period,
	 * marks the system as unhealthy with the computed severity and margin.
	 */
	void execute_use_case_safety_evaluation();

	/**
	 * @brief Publishes the safety evaluation result to uORB.
	 *
	 * Timestamps the output and publishes to ztss_use_case_motor_esc_failure.
	 */
	void publish_use_case_status();

private:

	// ── Publications ────────────────────────────────────────────────────
	uORB::Publication<ztss_monitor_use_case_output_s>
		ztss_use_case_motor_esc_failure_pub_{ORB_ID(ztss_use_case_motor_esc_failure)};

	// ── Subscriptions ───────────────────────────────────────────────────
	uORB::Subscription esc_status_sub_{ORB_ID(esc_status)};

	// ── Input messages ──────────────────────────────────────────────────
	esc_status_s     esc_status_input_{};

	// ── Output message ──────────────────────────────────────────────────
	ztss_monitor_use_case_output_s use_case_output_{};

	// ── Internal state ──────────────────────────────────────────────────
	bool updated_subscribers_{false};

	// Aggregate hysteresis for any-ESC-failure (debounce transient glitches)
	systemlib::Hysteresis esc_failure_hysteresis_{false};

	// Per-ESC tracking
	bool     esc_had_telemetry_[esc_status_s::CONNECTED_ESC_MAX]{};  // telemetry seen at least once
	uint8_t  esc_timed_out_mask_{0};    // bitmask: ESCs that have timed out
	uint8_t  esc_fault_mask_{0};        // bitmask: ESCs reporting failure flags
	uint8_t  failed_esc_count_{0};      // number of ESCs currently in failure

	// Computed per-cycle
	bool    any_esc_failure_{false};
	uint8_t severity_{ztss_monitor_use_case_output_s::INFO};
	uint8_t margin_{ztss_monitor_use_case_output_s::MARGIN_HIGH};
};
