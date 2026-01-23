#pragma once
#include <cstddef>
#include "actions.hpp"
#include "health_bits.hpp"

struct Rule {
  HealthMask required_faults;
  HealthMask required_ok;
  Action action;
  uint8_t priority;
};

// TODO(renzo) test case for each rule. SITL AND HITL
/*
Rule need to reflect the full system state of required ok flags and required faulty flags
*/
const Rule RULE_TABLE[2] = {

  {
    .required_faults = S_DUMMY_WARN | S_DUMMY_MARGIN_LOW,
    .required_ok     = 1ull,
    .action          = ACTION_HOLD,
    .priority        = 40
  },

  {
    .required_faults = S_DUMMY_CRITICAL | S_DUMMY_MARGIN_LOW,
    .required_ok     = 1ull,
    .action          = ACTION_LAND,
    .priority        = 40
  },

};


const size_t NUM_RULES = sizeof(RULE_TABLE)/sizeof(RULE_TABLE[0]);
