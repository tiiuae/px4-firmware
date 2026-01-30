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
  this->monitor_use_cases_subscriptions_ =
    std::array<uORB::Subscription,NUMBER_OF_MONITORING_CASES>{uORB::Subscription{ORB_ID(ztss_use_case_dummy)}};
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

  // use case --> dummy
  if (monitor_use_cases_subscriptions_[IDX_USE_CASE_DUMMY].updated())
  {
    ztss_monitor_use_case_output_s monitor_output;
    monitor_use_cases_subscriptions_[IDX_USE_CASE_DUMMY].copy(&monitor_output);

    fault_status = fault_status | make_fault_mask(monitor_output, IDX_USE_CASE_DUMMY);
    ok_status = ok_status | make_ok_mask(monitor_output, IDX_USE_CASE_DUMMY);
  }

  // use case --> b
  // use case --> c


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
