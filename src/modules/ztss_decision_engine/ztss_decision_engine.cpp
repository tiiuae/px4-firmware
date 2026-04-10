/*
❌ Reading uORB inside rule conditions
❌ Dynamic containers (std::vector)
❌ Scripted / runtime-loaded rules
❌ Mixing evaluation + execution
❌ Time-based logic inside conditions

❌ Calling orb_check() inside a rule
❌ Using Subscription::update() inside conditions
❌ Reading timestamps in conditions
❌ Mixing snapshot building with rule evaluation

static rules
pure conditions
priority-based arbitration
explicit actions
strict separation of concerns

Your implementation will look very similar to:
Failsafe::checkAndReport()
ModeChecks
HealthAndArmingChecks::update()


1. The Core Principle (PX4 Rule #1)
Rule evaluation must be pure and deterministic.
2. Pattern: “Snapshot Builder”.Build the Snapshot (Single Read Point)
3. Don’t Confuse Staleness with Validity
*/

#include "ztss_decision_engine.hpp"

// | → set union
// & → set intersection
// == → set inclusion test
// set max 64 bit
bool matches(const Rule &rule, const MonitorHealthMask &health_input)
{
  if(rule.required_faults == DEFAULT_FAULT_MASK) return false;
  // if(rule.required_ok == DEFAULT_OK_MASK) return false;

  return
    ((health_input.fault_mask & rule.required_faults) == rule.required_faults) &&
    ((health_input.ok_mask     & rule.required_ok)     == rule.required_ok);
};


HealthMask DecisionEngine::make_fault_mask(const ztss_monitor_use_case_output_s & monitor_output, size_t idx_use_case)
{
  HealthMask fault_maks = DEFAULT_FAULT_MASK;
	if ((monitor_output.timestamp - monitor_use_cases_last_timestamp_[idx_use_case])>250_ms) fault_maks = fault_maks | HealthFlagsCases[idx_use_case].S_NOT_SYNC;

  if(!monitor_output.healthy)
  {
    if (monitor_output.severity == ztss_monitor_use_case_output_s::WARN)      fault_maks = fault_maks | HealthFlagsCases[idx_use_case].S_WARN;
    if (monitor_output.severity == ztss_monitor_use_case_output_s::CRITICAL)  fault_maks = fault_maks | HealthFlagsCases[idx_use_case].S_CRITICAL;
    if (monitor_output.margin==ztss_monitor_use_case_output_s::MARGIN_LOW)    fault_maks = fault_maks | HealthFlagsCases[idx_use_case].S_MARGIN_LOW;
  }
	return fault_maks;
};
HealthMask DecisionEngine::make_ok_mask(const ztss_monitor_use_case_output_s & monitor_output, size_t idx_use_case)
{

  HealthMask ok_masks = DEFAULT_OK_MASK;

  if(monitor_output.healthy) ok_masks = ok_masks | HealthFlagsCases[idx_use_case].S_OK;

  return ok_masks;
}

DecisionEngine::DecisionEngine(): ModuleParams(nullptr)
{
  // set starting time of the event message
  event_.timestamp = hrt_absolute_time();
  for (size_t idx=0; idx< NUMBER_OF_MONITORING_CASES; idx++)
  {
    monitor_use_cases_last_timestamp_[idx] = uint64_t(0);
  }
  /* TOPICS
   ztss_use_case_attitude_failure
   ztss_use_case_ekf_divergence
   ztss_use_case_barometer_failure
   ztss_use_case_gps_failure
   ztss_use_case_imu_failure
   ztss_use_case_magnetometer_failure
   ztss_use_case_motor_esc_failure
   ztss_use_case_pose_estimate_lost
   ztss_use_case_motor_saturation
   ztss_use_case_vibration
  */
  this->monitor_use_cases_subscriptions_ =
    std::array<uORB::Subscription,NUMBER_OF_MONITORING_CASES>{
      uORB::Subscription{ORB_ID(ztss_use_case_attitude_failure)},
      uORB::Subscription{ORB_ID(ztss_use_case_ekf_divergence)},
      uORB::Subscription{ORB_ID(ztss_use_case_barometer_failure)},
      uORB::Subscription{ORB_ID(ztss_use_case_gps_failure)},
      uORB::Subscription{ORB_ID(ztss_use_case_imu_failure)},
      uORB::Subscription{ORB_ID(ztss_use_case_magnetometer_failure)},
      uORB::Subscription{ORB_ID(ztss_use_case_motor_esc_failure)},
      uORB::Subscription{ORB_ID(ztss_use_case_pose_estimate_lost)},
      uORB::Subscription{ORB_ID(ztss_use_case_motor_saturation)},
      uORB::Subscription{ORB_ID(ztss_use_case_vibration)}
    };
  this->ztss_event_pub_.advertise();
}

int DecisionEngine::task_spawn(int argc, char* argv[])
{
  _task_id = px4_task_spawn_cmd("ztss_decision_engine",
                                SCHED_DEFAULT,
                                SCHED_PRIORITY_DEFAULT + 40,
                                PX4_STACK_ADJUSTED(3250), // to be adjusted
                                (px4_main_t)&run_trampoline,
                                (char *const *)argv);

  if (_task_id<0)
  {
    _task_id=-1;
    return -errno;
  }

  if (wait_until_running() < 0)
  {
    _task_id = -1;
    return -1;
  }

  return 0;
}

DecisionEngine* DecisionEngine::instantiate(int argc, char *argv[])
{
  DecisionEngine* instance = new DecisionEngine();
  if (instance==nullptr)
  {
    PX4_ERR("Decision Engine nullptr");
  }
  return instance;
}

int DecisionEngine::custom_command(int argc, char *argv[]){return 0;}

int DecisionEngine::print_usage(const char *reason){return 0;}

void DecisionEngine::run()
{
  PX4_INFO("Ztss Decision Engine Started");

  while(!should_exit())
  {
    // run loop logic
    this->populate_use_cases_masks();
    this->evaluate_health_against_rules();
    this->publish_event_trigger();
    px4_usleep(100_ms); // 10 hz
  }
};

int DecisionEngine::print_status() {return 0;};

void DecisionEngine::populate_use_cases_masks()
{
  // reset health masks
  HealthMask fault_status = DEFAULT_FAULT_MASK;
  HealthMask ok_status = DEFAULT_OK_MASK;

  // 1 use case --> attitude failure
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_ATTITUDE_FAILURE].updated())
  {
    ztss_monitor_use_case_output_s monitor_output_attitude_failure;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_ATTITUDE_FAILURE].copy(&monitor_output_attitude_failure);

    fault_status = fault_status | make_fault_mask(monitor_output_attitude_failure, IDX_USE_CASE_ATTITUDE_FAILURE);
    ok_status = ok_status | make_ok_mask(monitor_output_attitude_failure, IDX_USE_CASE_ATTITUDE_FAILURE);
  }

  // 2 use case --> ekf divergence
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_EKF_DIVERGENCE].updated())
  {
    ztss_monitor_use_case_output_s monitor_output_ekf_divergence;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_EKF_DIVERGENCE].copy(&monitor_output_ekf_divergence);

    fault_status = fault_status | make_fault_mask(monitor_output_ekf_divergence, IDX_USE_CASE_EKF_DIVERGENCE);
    ok_status = ok_status | make_ok_mask(monitor_output_ekf_divergence, IDX_USE_CASE_EKF_DIVERGENCE);
  }

  // 3 use case --> barometer failure
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_BAROMETER_FAILURE].updated())
  {
    ztss_monitor_use_case_output_s monitor_output_barometer_failure;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_BAROMETER_FAILURE].copy(&monitor_output_barometer_failure);

    fault_status = fault_status | make_fault_mask(monitor_output_barometer_failure, IDX_USE_CASE_BAROMETER_FAILURE);
    ok_status = ok_status | make_ok_mask(monitor_output_barometer_failure, IDX_USE_CASE_BAROMETER_FAILURE);
  }

  // 4 use case --> gps failure
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_GPS_FAILURE].updated())
  {
    ztss_monitor_use_case_output_s monitor_output_gps_failure;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_GPS_FAILURE].copy(&monitor_output_gps_failure);

    fault_status = fault_status | make_fault_mask(monitor_output_gps_failure, IDX_USE_CASE_GPS_FAILURE);
    ok_status = ok_status | make_ok_mask(monitor_output_gps_failure, IDX_USE_CASE_GPS_FAILURE);
  }

  // 5 use case --> imu failure
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_IMU_FAILURE].updated())
  {
    ztss_monitor_use_case_output_s monitor_output_imu_failure;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_IMU_FAILURE].copy(&monitor_output_imu_failure);

    fault_status = fault_status | make_fault_mask(monitor_output_imu_failure, IDX_USE_CASE_IMU_FAILURE);
    ok_status = ok_status | make_ok_mask(monitor_output_imu_failure, IDX_USE_CASE_IMU_FAILURE);
  }

  // 6 use case --> magnetometer failure
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_MAGNETOMETER_FAILURE].updated())
  {
    ztss_monitor_use_case_output_s monitor_output_magnetometer_failure;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_MAGNETOMETER_FAILURE].copy(&monitor_output_magnetometer_failure);

    fault_status = fault_status | make_fault_mask(monitor_output_magnetometer_failure, IDX_USE_CASE_MAGNETOMETER_FAILURE);
    ok_status = ok_status | make_ok_mask(monitor_output_magnetometer_failure, IDX_USE_CASE_MAGNETOMETER_FAILURE);
  }

  // 7 use case --> motor/esc failure
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_MOTOR_ESC_FAILURE].updated())
  {
    ztss_monitor_use_case_output_s monitor_output_motor_esc_failure;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_MOTOR_ESC_FAILURE].copy(&monitor_output_motor_esc_failure);

    fault_status = fault_status | make_fault_mask(monitor_output_motor_esc_failure, IDX_USE_CASE_MOTOR_ESC_FAILURE);
    ok_status = ok_status | make_ok_mask(monitor_output_motor_esc_failure, IDX_USE_CASE_MOTOR_ESC_FAILURE);
  }

  // 8 use case --> pose estimate lost
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_POSE_ESTIMATE_FAILURE].updated())
  {
    ztss_monitor_use_case_output_s monitor_output_pose_estimate_failure;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_POSE_ESTIMATE_FAILURE].copy(&monitor_output_pose_estimate_failure);

    fault_status = fault_status | make_fault_mask(monitor_output_pose_estimate_failure, IDX_USE_CASE_POSE_ESTIMATE_FAILURE);
    ok_status = ok_status | make_ok_mask(monitor_output_pose_estimate_failure, IDX_USE_CASE_POSE_ESTIMATE_FAILURE);
  }

  this->monitor_status_masks_.ok_mask = ok_status;
  this->monitor_status_masks_.fault_mask = fault_status;
}

void DecisionEngine::evaluate_health_against_rules()
{
  uint8_t action_selected = ztss_event_trigger_s::ACTION_NONE;
  uint8_t best_prio = 0;

  // evaluation: NUM_RULES times.
  for (size_t idx_rule = 0; idx_rule< NUM_RULES; idx_rule++)
  {
      if (matches(RULE_TABLE[idx_rule], monitor_status_masks_)) {

        if (RULE_TABLE[idx_rule].priority > best_prio) {
          best_prio = RULE_TABLE[idx_rule].priority;
          action_selected = RULE_TABLE[idx_rule].action;
        }
      }
  }


  // Set the action to be taken
  event_.timestamp = hrt_absolute_time();
  event_.action_id = action_selected;
  event_.priority = best_prio;
}


void DecisionEngine::publish_event_trigger()
{
  // Check if for some reason the event timestamp has been left in the past
  uint64_t time_now = hrt_absolute_time();
  if ((time_now - event_.timestamp) > MAX_PERIOD_MS) return;

  // PX4_INFO("Triggered event %d", static_cast<int>(event_.action_id));

  ztss_event_pub_.publish(event_);
}

extern "C" __EXPORT int ztss_decision_engine_main(int argc, char* argv[]){return DecisionEngine::main(argc, argv);}
