

#include "ztss_attitude_failure.hpp"

ZtssCaseAttitudeFailure::ZtssCaseAttitudeFailure(ModuleParams *parent): ModuleParams(parent)
{
	ztss_use_case_attitude_failure_pub_.advertise();

}

// ─────────────────────────────────────────────────────────────────────────────
// 1.  Read the latest vehicle attitude from the estimator
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseAttitudeFailure::update_subscribed_values()
{

	if (vehicle_attitude_sub_.updated())
	{
		vehicle_attitude_sub_.update(&vehicle_attitude_input_);
		updated_subscriber_ = true;
		return;
	}
	updated_subscriber_ = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// 2.  Convert quaternion to Euler angles and evaluate severity levels
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseAttitudeFailure::construct_internal_bussiness_data()
{
	const matrix::Eulerf euler_angles = matrix::Quatf(this->vehicle_attitude_input_.q);
	float roll(euler_angles.phi());
	float pitch(euler_angles.theta());

	// get the angle limits
	const float max_roll_deg = _param_fd_fail_r.get();
	const float max_pitch_deg = _param_fd_fail_p.get();
	const float max_roll(fabsf(math::radians(max_roll_deg)));
	const float max_pitch(fabsf(math::radians(max_pitch_deg)));

	const bool roll_status = (max_roll > FLT_EPSILON) && (fabsf(roll) > max_roll);
	const bool pitch_status = (max_pitch > FLT_EPSILON) && (fabsf(pitch) > max_pitch);

	// TODO(renzo) check this logic!
	severity = ((fabsf(roll)) > max_roll - WARN_THRESHOLD || (fabsf(pitch)) > max_pitch - WARN_THRESHOLD)? ztss_monitor_use_case_output_s::WARN : ztss_monitor_use_case_output_s::INFO;
	severity = ((fabsf(roll)) > max_roll - CRITICAL_THRESHOLD || (fabsf(pitch)) > max_pitch - CRITICAL_THRESHOLD)? ztss_monitor_use_case_output_s::CRITICAL : ztss_monitor_use_case_output_s::WARN;

	switch (severity)
	{
	case ztss_monitor_use_case_output_s::WARN:
		margin_to_next_severity_case = (((fabsf(roll)-(max_roll-WARN_THRESHOLD))/(WARN_THRESHOLD - CRITICAL_THRESHOLD)) > MARGIN_LOW_CRITICAL_THRESHOLD || ((fabsf(pitch)-(max_pitch-WARN_THRESHOLD))/(WARN_THRESHOLD - CRITICAL_THRESHOLD)) > MARGIN_LOW_CRITICAL_THRESHOLD)? ztss_monitor_use_case_output_s::MARGIN_LOW:ztss_monitor_use_case_output_s::MARGIN_HIGH;
		break;
	case ztss_monitor_use_case_output_s::INFO:
		margin_to_next_severity_case = ((fabsf(roll)/(max_roll - WARN_THRESHOLD)) > MARGIN_LOW_WARN_THRESHOLD || (fabsf(pitch)/(max_pitch-WARN_THRESHOLD)) > MARGIN_LOW_WARN_THRESHOLD)? ztss_monitor_use_case_output_s::MARGIN_LOW:ztss_monitor_use_case_output_s::MARGIN_HIGH;
		break;
	default:
		break;
	}

	// Update hysteresis
	hrt_abstime time_now = hrt_absolute_time();

	roll_failure_hysteresis_.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_r_ttri.get()));
	pitch_failure_hysteresis_.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_p_ttri.get()));
	roll_failure_hysteresis_.set_state_and_update(roll_status, time_now);
	pitch_failure_hysteresis_.set_state_and_update(pitch_status, time_now);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3.  Evaluate the safety case based on hysteresis state
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseAttitudeFailure::execute_use_case_safety_evaluation()
{


	if (updated_subscriber_)
	{
		// failure case
		if (roll_failure_hysteresis_.get_state() || pitch_failure_hysteresis_.get_state())
		{
			this->use_case_output_.healthy=false;
			this->use_case_output_.severity = severity;
			this->use_case_output_.margin = margin_to_next_severity_case;
		}

		// ok case
		if (!roll_failure_hysteresis_.get_state() && !pitch_failure_hysteresis_.get_state())
		{
			this->use_case_output_.healthy = true;
			this->use_case_output_.severity = ztss_monitor_use_case_output_s::INFO;
			this->use_case_output_.margin = ztss_monitor_use_case_output_s::MARGIN_HIGH;
		}


	}
}

// ─────────────────────────────────────────────────────────────────────────────
// 4.  Publish the safety evaluation result
// ─────────────────────────────────────────────────────────────────────────────
void ZtssCaseAttitudeFailure::publish_use_case_status()
{
	this->use_case_output_.timestamp= hrt_absolute_time();
	this->ztss_use_case_attitude_failure_pub_.publish(this->use_case_output_);
}


