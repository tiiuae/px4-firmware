#pragma once
#include <cstddef>
#include <uORB/topics/ztss_event_trigger.h>
#include "actions.hpp"
#include "health_bits.hpp"

struct Rule {
  HealthMask required_faults;
  HealthMask required_ok;
  uint8_t action;
  uint8_t priority;
};

// TODO(renzo) test case for each rule. SITL AND HITL
/*
Rule need to reflect the full system state of required ok flags and required faulty flags
*/
const Rule RULE_TABLE[4] = {
  // 1.a ATTITUDE FAILURE WARN - MARGIN LOW
  {
    .required_faults = S_ATTITUDE_FAILURE_WARN | S_ATTITUDE_FAILURE_MARGIN_LOW,
    .required_ok= H_ATT_EST_OK | H_MOTORS_OK,
    .action=ztss_event_trigger_s::ACTION_HOLD_ATTITUDE,
    .priority=50
  },
  // 1.b ATTITUDE FAILURE CRITICAL
  {
    .required_faults = S_ATTITUDE_FAILURE_CRITICAL,
    .required_ok= 0ull,
    .action= ztss_event_trigger_s::ACTION_PARACHUTE,
    .priority=75
  },
  // 2.a EKF DIVERGENCE WARN
  {
    .required_faults = S_EKF_DIVERGENCE_FAILURE_WARN | S_EKF_DIVERGENCE_FAILURE_MARGIN_LOW,
    .required_ok= H_POS_EST_OK | H_MOTORS_OK,
    .action= ztss_event_trigger_s::ACTION_HOLD_POSITION,
    .priority=49
  },
  // 2.b EKF DIVERGENCE CRITICAL
  {
    .required_faults = S_EKF_DIVERGENCE_FAILURE_CRITICAL,
    .required_ok = H_POS_EST_OK | H_GPS_OK | H_MOTORS_OK,
    .action=ztss_event_trigger_s::ACTION_HOLD_POSITION,
    .priority=74
  },


};


constexpr size_t NUM_RULES = sizeof(RULE_TABLE)/sizeof(RULE_TABLE[0]);
