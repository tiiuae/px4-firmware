/*

A Runtime Assurance Supervisor with hierarchical safety authority

⚠ Commander-Adjacent Module (Advanced / Careful)
When to use
experimental logic
research builds
temporary validation
Pattern
decision_engine_module
    ↓ publishes health / fault topic
Commander
    ↓ converts to failsafe_flags

Rules
module publishes facts
Commander remains final authority

⚠ Still acceptable
❌ Do NOT publish failsafe_flags directly
❌ Do NOT send LAND / RTL commands
❌ Standalone Decision Engine (Wrong)

This will fight PX4:

publishes vehicle_command

switches nav modes

duplicates failsafe logic

❌ Conflicts with Commander
❌ Breaks parameters
❌ Hard to debug
❌ Unsafe

Actions Commander Can Trigger
LAND
RTL
HOLD
POSCTL
DISARM
TERMINATE
Mode fallback (OFFBOARD → POSCTL)

2.4 Baseline Controllers (BC)
RTA Definition

A controller that is provably safe, possibly conservative, and assumed to always maintain system safety.

Your Baseline Controllers
| Action       | RTA Role                          |
| ------------ | --------------------------------- |
| RTL          | Mission-level safe controller     |
| LAND         | Energy-minimizing safe controller |
| PARACHUTE    | Catastrophic-safe controller      |
| M33 external | Diverse-safe controller           |


Health Inputs
| uORB Topic         | Purpose                  |
| ------------------ | ------------------------ |
| `failsafe_flags`   | Aggregated system faults |
| `battery_status`   | Low / critical battery   |
| `estimator_status` | EKF validity             |
| `sensor_preflight` | Sensor health            |
| `actuator_failure` | Motor/servo faults       |

Availability Inputs
| Topic                   | Purpose                       |
| ----------------------- | ----------------------------- |
| `rc_channels`           | RC availability               |
| `offboard_control_mode` | Offboard signal presence      |
| `vehicle_control_mode`  | Which control loops are valid |

State Checks Input
| Topic                   | Purpose                |
| ----------------------- | ---------------------- |
| `vehicle_status`        | Armed state, nav state |
| `vehicle_land_detected` | On ground / landed     |
| `home_position`         | Required for RTL       |

Parameters used in Commander

| Parameter         | Example                |
| ----------------- | ---------------------- |
| `COM_FAIL_ACT_T`  | Data link loss action  |
| `COM_OBL_RC_ACT`  | Offboard loss behavior |
| `NAV_DLL_ACT`     | RC failsafe            |
| `RTL_RETURN_ALT`  | RTL altitude           |
| `COM_LOW_BAT_ACT` | Battery failsafe       |

COMPARISON

| Old Commander Role  | New Role         |
| ------------------- | ---------------- |
| Detect faults       | Monitoring cases |
| Decide action       | Decision Engine  |
| Execute soft safety | Commander        |
| Execute hard safety | Safety layer     |

Commander becomes a subordinate safety executor.
5. Scheduling & Priority (Critical)

Your safety stack must:
run faster or equal to Commander
never block
preempt normal control paths if needed

Typical:

Work queue with higher priority
Fixed cycle time
Bounded execution


| Failure                 | Required Behavior           |
| ----------------------- | --------------------------- |
| Decision engine crashes | Parachute or M33 intervenes |
| RPMsg fails             | FMU still deploys parachute |
| Commander stalls        | Hard safety bypass          |
| Monitor false positive  | Conservative but reversible |


7. Migration Strategy (Do NOT Skip This)
Phase 1 — Shadow Mode
Monitoring + Decision Engine
Log actions only
Commander still in charge
Commander still in charge

Phase 2 — Soft Override
Decision Engine requests LAND / RTL
Commander still decides

Phase 3 — Hard Safety Enabled
Parachute + RPMsg active
Commander becomes optional

Phase 4 — Certification / Validation
Prove:
bounded execution
dominance of hard safety
no deadlocks


Bounded execution means:
The decision engine must complete in a known, deterministic upper time bound,
regardless of inputs.
Why this matters in PX4

PX4 runs:
In NuttX
With hard real-time constraints
Inside work queues / scheduler loops

A decision engine that:
Iterates dynamically
Allocates memory
Traverses variable-length containers
Evaluates complex logic trees
…can stall the flight stack.

What do implement:
✔ Fixed loop
✔ No allocations
✔ No recursion
✔ Constant memory
✔ Constant time

RTA requires:
Deterministic response time
Verifiable worst-case execution time (WCET)

Autority Transfer
| Situation    | Authority                          |
| ------------ | ---------------------------------- |
| Normal       | Commander + Navigator              |
| Degraded     | Decision Engine requests RTL       |
| Severe       | Decision Engine triggers parachute |
| Catastrophic | M33 autonomous recovery            |

Mapping back to Simplex / RTA

| Message field | RTA meaning                 |
| ------------- | --------------------------- |
| `healthy`     | Inside invariant            |
| `severity`    | Degree of violation         |
| `margin`      | Distance to safety boundary |
| `timestamp`   | Validity of observation     |

*/
#pragma once
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/critical_action.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <array>
#include "helpers/mask_helper.h"
#include <uORB/topics/ztss_monitor_use_case_output.h> 	// Subscription
#include "uORB/topics/ztss_event_trigger.h"		// Publication





using namespace time_literals;
using timestamp_time = uint64_t;
constexpr int NUMBER_OF_MONITORING_CASES = 1;
constexpr size_t MAX_PERIOD_MS = 100_ms;

class DecisionEngine : public ModuleBase<DecisionEngine>, public ModuleParams
{
public:
	DecisionEngine();
	~DecisionEngine()=default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static DecisionEngine *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void evaluate_health_against_rules() ;

	void populate_use_cases_masks();

	void publish_event_trigger();


	HealthMask make_fault_mask(const ztss_monitor_use_case_output_s & , size_t );
	HealthMask make_ok_mask(const ztss_monitor_use_case_output_s & , size_t );




private:

std::array<uORB::Subscription, NUMBER_OF_MONITORING_CASES> 	monitor_use_cases_subscriptions_;

MonitorHealthMask monitor_status_masks_;
std::array<timestamp_time, NUMBER_OF_MONITORING_CASES> 		monitor_use_cases_last_timestamp_;

uORB::Publication<ztss_event_trigger_s> ztss_event_pub_{ORB_ID(ztss_event)};

ztss_event_trigger_s event_{};

};
