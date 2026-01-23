#pragma once
#include <cstdint>
using HealthMask = uint64_t;
struct MonitorHealthMask {
  HealthMask ok_mask;      // bits asserted as healthy
  HealthMask fault_mask;   // bits asserted as faulty
};
