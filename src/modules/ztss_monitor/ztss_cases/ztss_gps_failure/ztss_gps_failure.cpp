#include "ztss_gps_failure.hpp"
#include <float.h>
#include <math.h>

ZtssCaseGpsFailure::ZtssCaseGpsFailure(ModuleParams *parent)
	: ModuleParams(parent)
{
	ztss_use_case_gps_failure_pub_.advertise();
	gps_failure_hysteresis_.set_hysteresis_time_from(false, GPS_FAILURE_HYSTERESIS_US);
}

// ─────────────────────────────────────────────────────────────────────────────
// 1.  Read the latest GPS, GNSS relative, and satellite info data
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseGpsFailure::update_subscribed_values()
{
	bool any_updated = false;

	if (sensor_gps_sub_.updated()) {
		sensor_gps_sub_.copy(&sensor_gps_input_);
		any_updated = true;
	}

	if (sensor_gnss_relative_sub_.updated()) {
		sensor_gnss_relative_sub_.copy(&sensor_gnss_relative_input_);
		gnss_relative_available_ = true;
		// Note: gnss_relative updates are less frequent; we don't gate on it
	}

	if (satellite_info_sub_.updated()) {
		satellite_info_sub_.copy(&satellite_info_input_);
	}

	updated_subscribers_ = any_updated;
}

// ─────────────────────────────────────────────────────────────────────────────
// 2.  Evaluate GPS health: fix quality, accuracy, satellites, interference
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseGpsFailure::construct_internal_business_data()
{
	const hrt_abstime time_now = hrt_absolute_time();

	// Read EKF2 GPS quality thresholds from parameters
	const float eph_threshold     = _param_ekf2_req_eph.get();     // metres
	const float epv_threshold     = _param_ekf2_req_epv.get();     // metres
	const float sacc_threshold    = _param_ekf2_req_sacc.get();    // m/s
	const int   nsats_threshold   = _param_ekf2_req_nsats.get();   // count
	const float pdop_threshold    = _param_ekf2_req_pdop.get();    // dimensionless

	// ── Check 1: Fix type ───────────────────────────────────────────────
	const uint8_t fix_type = sensor_gps_input_.fix_type;
	fix_type_failure_ = (fix_type < GPS_MIN_FIX_TYPE_3D);

	// ── Check 2: Satellite count ────────────────────────────────────────
	const uint8_t satellites_used = sensor_gps_input_.satellites_used;
	satellite_count_failure_ = (satellites_used < nsats_threshold);

	// ── Check 3: Horizontal position accuracy (eph) ─────────────────────
	const float current_eph = sensor_gps_input_.eph;
	eph_failure_ = (eph_threshold > FLT_EPSILON) && (current_eph > eph_threshold);

	// ── Check 4: Vertical position accuracy (epv) ───────────────────────
	const float current_epv = sensor_gps_input_.epv;
	epv_failure_ = (epv_threshold > FLT_EPSILON) && (current_epv > epv_threshold);

	// ── Check 5: Speed accuracy ─────────────────────────────────────────
	const float current_sacc = sensor_gps_input_.s_variance_m_s;
	speed_accuracy_failure_ = (sacc_threshold > FLT_EPSILON) && (current_sacc > sacc_threshold);

	// ── Check 6: PDOP (compute from hdop and vdop) ──────────────────────
	// PDOP** = HDOP** + VDOP**
	const float hdop = sensor_gps_input_.hdop;
	const float vdop = sensor_gps_input_.vdop;
	const float pdop = sqrtf(hdop * hdop + vdop * vdop);
	pdop_failure_ = (pdop_threshold > FLT_EPSILON) && (pdop > pdop_threshold);

	// ── Check 7: Jamming detection ──────────────────────────────────────
	// jamming_state: 0=Unknown, 1=OK, 2=Warning, 3=Critical
	jamming_detected_ = (sensor_gps_input_.jamming_state >= sensor_gps_s::JAMMING_STATE_WARNING);

	// ── Check 8: Spoofing detection ─────────────────────────────────────
	// spoofing_state: 0=Unknown, 1=None, 2=Indicated, 3=Multiple
	spoofing_detected_ = (sensor_gps_input_.spoofing_state >= sensor_gps_s::SPOOFING_STATE_INDICATED);

	// ── Check 9: GPS data staleness ─────────────────────────────────────
	const hrt_abstime gps_age = time_now - sensor_gps_input_.timestamp;
	gps_stale_ = (sensor_gps_input_.timestamp > 0) && (gps_age > GPS_STALENESS_TIMEOUT_US);

	// ── Check 10: GNSS relative fix lost ────────────────────────────────
	// Only evaluate if we have ever received gnss_relative data
	if (gnss_relative_available_) {
		gnss_relative_fix_lost_ = !sensor_gnss_relative_input_.gnss_fix_ok;
	} else {
		gnss_relative_fix_lost_ = false;
	}



	// ── Severity computation ────────────────────────────────────────────
	//
	// CRITICAL conditions (GPS unusable):
	//   - No fix or fix < 2D
	//   - GPS data stale (no updates)
	//   - Spoofing with multiple indicators
	//   - Jamming at critical level
	//   - Both eph AND satellite count failed simultaneously
	//
	// WARN conditions (GPS degraded):
	//   - Fix type is 2D (no altitude from GPS)
	//   - Satellite count below threshold
	//   - eph or epv exceeded
	//   - Speed accuracy exceeded
	//   - PDOP exceeded
	//   - Jamming at warning level
	//   - Spoofing indicated
	//   - GNSS relative fix lost
	//   - SNR degraded
	//
	// INFO: All checks pass

	uint8_t computed_severity = ztss_monitor_use_case_output_s::INFO;
	uint8_t computed_margin   = ztss_monitor_use_case_output_s::MARGIN_HIGH;

	// ── Critical conditions ─────────────────────────────────────────────
	if (gps_stale_
	    || (fix_type <= sensor_gps_s::FIX_TYPE_NONE)    // No fix at all
	    || (sensor_gps_input_.jamming_state == sensor_gps_s::JAMMING_STATE_CRITICAL)
	    || (sensor_gps_input_.spoofing_state == sensor_gps_s::SPOOFING_STATE_MULTIPLE)
	    || (eph_failure_ && satellite_count_failure_)    // Combined degradation → unusable
	    ) {

		computed_severity = ztss_monitor_use_case_output_s::CRITICAL;
		computed_margin   = ztss_monitor_use_case_output_s::MARGIN_LOW;

	// ── Warning conditions ──────────────────────────────────────────────
	} else if (fix_type_failure_
		   || satellite_count_failure_
		   || eph_failure_
		   || epv_failure_
		   || speed_accuracy_failure_
		   || pdop_failure_
		   || jamming_detected_
		   || spoofing_detected_
		   || gnss_relative_fix_lost_
		   ) {

		computed_severity = ztss_monitor_use_case_output_s::WARN;

		// Compute margin: how many warning checks are active at once
		// More simultaneous warnings → closer to CRITICAL (lower margin)
		uint8_t warning_count = 0;
		warning_count += fix_type_failure_ ? 1 : 0;
		warning_count += satellite_count_failure_ ? 1 : 0;
		warning_count += eph_failure_ ? 1 : 0;
		warning_count += epv_failure_ ? 1 : 0;
		warning_count += speed_accuracy_failure_ ? 1 : 0;
		warning_count += pdop_failure_ ? 1 : 0;
		warning_count += jamming_detected_ ? 1 : 0;
		warning_count += spoofing_detected_ ? 1 : 0;
		warning_count += gnss_relative_fix_lost_ ? 1 : 0;

		// 3+ simultaneous warnings → MARGIN_LOW (approaching CRITICAL)
		computed_margin = (warning_count >= 3)
				  ? ztss_monitor_use_case_output_s::MARGIN_LOW
				  : ztss_monitor_use_case_output_s::MARGIN_HIGH;

	// ── INFO: all checks pass ───────────────────────────────────────────
	} else {
		computed_severity = ztss_monitor_use_case_output_s::INFO;

		// Compute margin from eph approaching the threshold
		if (eph_threshold > FLT_EPSILON) {
			const float eph_ratio = current_eph / eph_threshold;

			if (eph_ratio > GPS_EPH_WARN_FRACTION) {
				// Approaching eph limit — compute progress within the approach zone
				const float warn_band = 1.0f - GPS_EPH_WARN_FRACTION;
				const float progress = (warn_band > FLT_EPSILON)
						       ? ((eph_ratio - GPS_EPH_WARN_FRACTION) / warn_band)
						       : 1.0f;

				computed_margin = (progress > GPS_MARGIN_LOW_THRESHOLD)
						  ? ztss_monitor_use_case_output_s::MARGIN_LOW
						  : ztss_monitor_use_case_output_s::MARGIN_HIGH;
			}
		}

		// Also check satellite count approaching minimum
		if (nsats_threshold > 0 && computed_margin == ztss_monitor_use_case_output_s::MARGIN_HIGH) {
			// Warn when only 1–2 satellites above minimum
			const int satellite_margin_count = static_cast<int>(satellites_used) - nsats_threshold;

			if (satellite_margin_count <= 2 && satellite_margin_count >= 0) {
				computed_margin = ztss_monitor_use_case_output_s::MARGIN_LOW;
			}
		}
	}

	gps_failure_ = (computed_severity >= ztss_monitor_use_case_output_s::WARN);
	severity_ = computed_severity;
	margin_   = computed_margin;

	// Feed the aggregate failure state into hysteresis
	gps_failure_hysteresis_.set_state_and_update(gps_failure_, time_now);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3.  Evaluate the safety case based on hysteresis state
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseGpsFailure::execute_use_case_safety_evaluation()
{
	if (!updated_subscribers_) {
		return;
	}

	if (gps_failure_hysteresis_.get_state()) {
		// GPS failure has persisted beyond the hysteresis period
		use_case_output_.healthy  = false;
		use_case_output_.severity = severity_;
		use_case_output_.margin   = margin_;

	} else {
		// GPS is healthy (or transient degradation cleared)
		use_case_output_.healthy  = true;
		use_case_output_.severity = severity_;
		use_case_output_.margin   = margin_;
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// 4.  Publish the safety evaluation result
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseGpsFailure::publish_use_case_status()
{
	use_case_output_.timestamp = hrt_absolute_time();
	ztss_use_case_gps_failure_pub_.publish(use_case_output_);
}
