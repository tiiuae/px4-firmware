#include "ztss_motor_saturation.hpp"

ZtssCaseMotorSaturation::ZtssCaseMotorSaturation(ModuleParams *parent)
	: ModuleParams(parent),
	  motor_upper_sat_hysteresis_{
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false),
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false)
	  },
	  motor_lower_sat_hysteresis_{
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false),
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false)
	  },
	  motor_sat_warn_hysteresis_{
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false),
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false)
	  },
	  motor_sat_margin_low_hysteresis_{
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false),
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false)
	  },
	  motor_sat_critical_hysteresis_{
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false),
		  systemlib::Hysteresis(false), systemlib::Hysteresis(false)
	  }
{
	ztss_use_case_motor_saturation_pub_.advertise();

	// Configure hysteresis debounce times for all motor outputs
	for (uint8_t i = 0; i < MOTOR_SAT_MAX_NUM_OUTPUTS; ++i) {
		// Direction-detection hysteresis (instantaneous, no debounce)
		motor_upper_sat_hysteresis_[i].set_hysteresis_time_from(false, 0);
		motor_lower_sat_hysteresis_[i].set_hysteresis_time_from(false, 0);

		// Severity / margin hysteresis (time-based thresholds)
		motor_sat_warn_hysteresis_[i].set_hysteresis_time_from(false, MOTOR_SAT_WARN_HYSTERESIS_US);
		motor_sat_margin_low_hysteresis_[i].set_hysteresis_time_from(false, MOTOR_SAT_MARGIN_LOW_HYSTERESIS_US);
		motor_sat_critical_hysteresis_[i].set_hysteresis_time_from(false, MOTOR_SAT_CRITICAL_HYSTERESIS_US);
	}
}


// 1.  Read the latest actuator outputs
void ZtssCaseMotorSaturation::update_subscribed_values()
{
	if (actuator_outputs_sub_.updated()) {
		actuator_outputs_sub_.update(&actuator_outputs_input_);
		updated_subscriber_ = true;
		return;
	}

	updated_subscriber_ = false;
}

// 2.  Evaluate per-motor saturation and compute severity / margin
void ZtssCaseMotorSaturation::construct_internal_business_data()
{
	const hrt_abstime time_now = hrt_absolute_time();

	bool any_warn     = false;
	bool any_margin_low = false;
	bool any_critical  = false;

	for (uint8_t i = 0; i < MOTOR_SAT_MAX_NUM_OUTPUTS; ++i) {
		const float output_val = actuator_outputs_input_.output[i];

		//  Direction detection
		const bool upper_saturated = (output_val >= MOTOR_SATURATION_UPPER_LIMIT);
		const bool lower_saturated = (output_val <= MOTOR_SATURATION_LOWER_LIMIT);

		motor_upper_sat_hysteresis_[i].set_state_and_update(upper_saturated, time_now);
		motor_lower_sat_hysteresis_[i].set_state_and_update(lower_saturated, time_now);

		// Combined saturation flag (either direction)
		const bool is_saturated = upper_saturated || lower_saturated;

		//  Duration-based severity hysteresis
		motor_sat_warn_hysteresis_[i].set_state_and_update(is_saturated, time_now);       // 300 ms
		motor_sat_margin_low_hysteresis_[i].set_state_and_update(is_saturated, time_now); // 600 ms
		motor_sat_critical_hysteresis_[i].set_state_and_update(is_saturated, time_now);   // 1 s

		if (motor_sat_critical_hysteresis_[i].get_state()) {
			any_critical = true;
		}

		if (motor_sat_warn_hysteresis_[i].get_state()) {
			any_warn = true;
		}

		// Give a margin when the motor is in a warning state
		if (motor_sat_warn_hysteresis_[i].get_state() && motor_sat_margin_low_hysteresis_[i].get_state()) {
			any_margin_low = true;
		}


	}

	//  Severity computation (based on saturation duration)
	if (any_critical) {
		severity_ = ztss_monitor_use_case_output_s::CRITICAL;

	} else if (any_warn) {
		severity_ = ztss_monitor_use_case_output_s::WARN;

	} else {
		severity_ = ztss_monitor_use_case_output_s::INFO;
	}

	//  Margin computation (600 ms → approaching CRITICAL)
	if (any_margin_low) {
		margin_ = ztss_monitor_use_case_output_s::MARGIN_LOW;

	} else {
		margin_ = ztss_monitor_use_case_output_s::MARGIN_HIGH;
	}
}

// 3.  Evaluate the safety case based on hysteresis state
void ZtssCaseMotorSaturation::execute_use_case_safety_evaluation()
{
	if (updated_subscriber_) {
		if (severity_ > ztss_monitor_use_case_output_s::INFO) {
			// At least one motor saturated beyond WARN threshold
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

// 4.  Publish the safety evaluation result
void ZtssCaseMotorSaturation::publish_use_case_status()
{
	use_case_output_.timestamp = hrt_absolute_time();
	ztss_use_case_motor_saturation_pub_.publish(use_case_output_);
}
