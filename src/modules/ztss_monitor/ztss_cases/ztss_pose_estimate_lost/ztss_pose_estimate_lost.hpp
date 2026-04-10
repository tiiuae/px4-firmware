#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ztss_monitor_use_case_output.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_status.h>
#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

/**
 * @brief ZTSS Safety Case #4 – Position estimate lost
 *
 * Monitors the vehicle's position estimate validity using the
 * vehicle_local_position and estimator_status uORB topics.
 *
 * The vehicle_local_position topic is published by the EKF2 estimator and
 * contains validity flags (xy_valid, z_valid, v_xy_valid, v_z_valid),
 * accuracy estimates (eph, epv), and a dead-reckoning flag.
 *
 * Detection checks:
 *   1. Horizontal position invalid   — xy_valid = false
 *   2. Vertical position invalid     — z_valid = false
 *   3. Heading not good for control  — heading_good_for_control = false
 *   4. Position accuracy degraded    — eph exceeds COM_POS_FS_EPH threshold
 *   5. Timestamp stale               — topic not updated within COM_POS_FS_DELAY
 *   6. Dead-reckoning only           — xy_global = false (no absolute reference)
 *
 * Safe action: Land in attitude mode (no position hold possible).
 */

// Hysteresis hold time before confirming position estimate loss
static constexpr hrt_abstime POSE_LOST_HYSTERESIS_US = 1_s;

// Fraction of eph threshold at which we enter WARN band
static constexpr float POSE_EPH_WARN_FRACTION  = 0.7f;

// Margin threshold within each severity band
static constexpr float POSE_MARGIN_LOW_THRESHOLD = 0.75f;


class ZtssCasePoseEstimateLost : public ModuleParams
{
public:
	/**
	 * @brief Constructor — initializes the pose estimate lost monitor case.
	 * @param parent Pointer to parent ModuleParams for parameter inheritance.
	 */
	ZtssCasePoseEstimateLost(ModuleParams *parent);
	~ZtssCasePoseEstimateLost() = default;

	/**
	 * @brief Polls subscribed uORB topics for new position estimate data.
	 *
	 * Updates vehicle_local_position and estimator_status messages.
	 * Sets updated_subscribers_ flag if any new data was received.
	 */
	void update_subscribed_values();

	/**
	 * @brief Evaluates position estimate validity and accuracy.
	 *
	 * Checks validity flags (xy_valid, z_valid, heading_good_for_control),
	 * position accuracy (eph vs COM_POS_FS_EPH), timestamp staleness
	 * (vs COM_POS_FS_DELAY), and dead-reckoning status. Computes aggregate
	 * severity and margin, then feeds the hysteresis filter.
	 */
	void construct_internal_business_data();

	/**
	 * @brief Executes the safety evaluation based on hysteresis-debounced state.
	 *
	 * If position estimate has been lost longer than the hysteresis period,
	 * marks the system as unhealthy with the computed severity and margin.
	 */
	void execute_use_case_safety_evaluation();

	/**
	 * @brief Publishes the safety evaluation result to uORB.
	 *
	 * Timestamps the output and publishes to ztss_use_case_pose_estimate_lost.
	 */
	void publish_use_case_status();

private:

	// ── Publications ────────────────────────────────────────────────────
	uORB::Publication<ztss_monitor_use_case_output_s>
		ztss_use_case_pose_estimate_lost_pub_{ORB_ID(ztss_use_case_pose_estimate_lost)};

	// ── Subscriptions ───────────────────────────────────────────────────
	uORB::Subscription vehicle_local_position_sub_{ORB_ID(vehicle_local_position)};
	uORB::Subscription estimator_status_sub_{ORB_ID(estimator_status)};

	// ── Input messages ──────────────────────────────────────────────────
	vehicle_local_position_s vehicle_local_position_input_{};
	estimator_status_s       estimator_status_input_{};

	// ── Output message ──────────────────────────────────────────────────
	ztss_monitor_use_case_output_s use_case_output_{};

	// ── Internal state ──────────────────────────────────────────────────
	bool updated_subscribers_{false};

	// Hysteresis for position estimate loss (debounce transient drops)
	systemlib::Hysteresis position_lost_hysteresis_{false};

	// Per-cycle computed state
	bool    position_estimate_lost_{false};
	uint8_t severity_{ztss_monitor_use_case_output_s::INFO};
	uint8_t margin_{ztss_monitor_use_case_output_s::MARGIN_HIGH};

	// ── Parameters (re-use existing PX4 position failsafe thresholds) ──
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::COM_POS_FS_EPH>) _param_com_pos_fs_eph,   // max acceptable eph (m)
		(ParamInt<px4::params::COM_POS_FS_DELAY>) _param_com_pos_fs_delay  // max acceptable staleness (ms)
	)
};
