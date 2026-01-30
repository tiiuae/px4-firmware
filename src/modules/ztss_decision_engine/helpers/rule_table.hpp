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
const Rule RULE_TABLE[3] = {

  {
    .required_faults = S_DUMMY_WARN,
    .required_ok     = 0ull,
    .action          = ztss_event_trigger_s::ACTION_HOLD_POSITION,
    .priority        = 10
  },

  {
    .required_faults = S_DUMMY_WARN | S_DUMMY_MARGIN_LOW,
    .required_ok     = 0ull,
    .action          = ztss_event_trigger_s::ACTION_HOLD_ATTITUDE,
    .priority        = 50
  },

  {
    .required_faults = S_DUMMY_CRITICAL,
    .required_ok     = 0ull,
    .action          = ztss_event_trigger_s::ACTION_PARACHUTE,
    .priority        = 100
  }

};


const size_t NUM_RULES = sizeof(RULE_TABLE)/sizeof(RULE_TABLE[0]);
