#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/actuator_outputs.h>
#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

/**
 * @brief ZTSS Safety Case – Motor Output Saturation
 *
 * Monitors the actuator_outputs uORB topic for motor output saturation.
 * Actuator outputs are in the normalised range [0, 1].  A motor is
 * considered "saturated" when its output reaches the upper limit
 * (>= UPPER_LIMIT, commanding maximum thrust) or falls to the lower
 * limit (<= LOWER_LIMIT, commanding near-zero thrust), indicating
 * potential loss of control authority.
 *
 * Each motor has two direction-detection hysteresis variables:
 *   - Upper saturation hysteresis: output >= UPPER_LIMIT
 *   - Lower saturation hysteresis: output <= LOWER_LIMIT
 *
 * Severity is determined by *duration* of sustained saturation:
 *   WARN     — any motor saturated for > 300 ms
 *   CRITICAL — any motor saturated for > 1 s
 *
 * Margin:
 *   MARGIN_LOW  — any motor saturated for > 600 ms (approaching CRITICAL)
 *   MARGIN_HIGH — otherwise
 */

/// Maximum number of motor outputs to monitor (propeller motors only)
static constexpr uint8_t MOTOR_SAT_MAX_NUM_OUTPUTS = 4;

/// Upper saturation limit (normalised 0-1 output)
static constexpr float MOTOR_SATURATION_UPPER_LIMIT = 1.0f;

/// Lower saturation limit (normalised 0-1 output)
static constexpr float MOTOR_SATURATION_LOWER_LIMIT = 0.0f;

/// Hysteresis time for WARN detection (300 ms)
static constexpr hrt_abstime MOTOR_SAT_WARN_HYSTERESIS_US     = 300_ms;

/// Hysteresis time for MARGIN_LOW detection (600 ms)
static constexpr hrt_abstime MOTOR_SAT_MARGIN_LOW_HYSTERESIS_US = 600_ms;

/// Hysteresis time for CRITICAL detection (1 s)
static constexpr hrt_abstime MOTOR_SAT_CRITICAL_HYSTERESIS_US = 1000_ms;


class ZtssCaseMotorSaturation : public ModuleParams
{
public:
	/**
	 * @brief Constructor — initializes the motor saturation monitor case.
	 * @param parent Pointer to parent ModuleParams for parameter inheritance.
	 */
	ZtssCaseMotorSaturation(ModuleParams *parent);
	~ZtssCaseMotorSaturation() = default;

	/**
	 * @brief Polls subscribed uORB topics for new actuator output data.
	 *
	 * Updates actuator_outputs_input_ from the actuator_outputs topic.
	 * Sets updated_subscriber_ flag if new data was received.
	 */
	void update_subscribed_values();

	/**
	 * @brief Evaluates per-motor output saturation from actuator data.
	 *
	 * For each active motor output: checks whether the value exceeds the
	 * upper saturation limit or falls below the lower saturation limit.
	 * Feeds per-motor boolean states into hysteresis filters. Computes
	 * aggregate severity (CRITICAL/WARN/INFO) and margin.
	 */
	void construct_internal_business_data();

	/**
	 * @brief Executes the safety evaluation based on hysteresis-debounced state.
	 *
	 * If any motor has been in saturation longer than the hysteresis period,
	 * marks the system as unhealthy with the computed severity and margin.
	 */
	void execute_use_case_safety_evaluation();

	/**
	 * @brief Publishes the safety evaluation result to uORB.
	 *
	 * Timestamps the output and publishes to ztss_use_case_motor_saturation.
	 */
	void publish_use_case_status();

private:

	//  Publications
	uORB::Publication<ztss_monitor_use_case_output_s> ztss_use_case_motor_saturation_pub_{ORB_ID(ztss_use_case_motor_saturation)};

	//  Subscriptions
	uORB::Subscription actuator_outputs_sub_{ORB_ID(actuator_outputs)};

	//  Input messages
	actuator_outputs_s actuator_outputs_input_{};

	//  Output message
	ztss_monitor_use_case_output_s use_case_output_{};

	//  Internal state
	bool updated_subscriber_{false};

	/// Per-motor hysteresis for upper saturation detection (output >= UPPER_LIMIT)
	systemlib::Hysteresis motor_upper_sat_hysteresis_[MOTOR_SAT_MAX_NUM_OUTPUTS];

	/// Per-motor hysteresis for lower saturation detection (output <= LOWER_LIMIT)
	systemlib::Hysteresis motor_lower_sat_hysteresis_[MOTOR_SAT_MAX_NUM_OUTPUTS];

	/// Per-motor hysteresis for WARN severity (300 ms sustained saturation)
	systemlib::Hysteresis motor_sat_warn_hysteresis_[MOTOR_SAT_MAX_NUM_OUTPUTS];

	/// Per-motor hysteresis for MARGIN_LOW (600 ms sustained saturation)
	systemlib::Hysteresis motor_sat_margin_low_hysteresis_[MOTOR_SAT_MAX_NUM_OUTPUTS];

	/// Per-motor hysteresis for CRITICAL severity (1 s sustained saturation)
	systemlib::Hysteresis motor_sat_critical_hysteresis_[MOTOR_SAT_MAX_NUM_OUTPUTS];

	// Computed per-cycle
	uint8_t severity_{ztss_monitor_use_case_output_s::INFO};
	uint8_t margin_{ztss_monitor_use_case_output_s::MARGIN_HIGH};
};
