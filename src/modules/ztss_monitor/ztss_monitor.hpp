#pragma once

#include "worker_thread.hpp"
#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/critical_action.h>

#include "ztss_cases/ztss_attitude_failure/ztss_attitude_failure.hpp"
#include "ztss_cases/ztss_ekf_divergence/ztss_ekf_divergence.hpp"
#include "ztss_cases/ztss_imu_failure/ztss_imu_failure.hpp"
#include "ztss_cases/ztss_magnetometer_failure/ztss_magnetometer_failure.hpp"
#include "ztss_cases/ztss_barometer_failure/ztss_barometer_failure.hpp"
#include "ztss_cases/ztss_gps_failure/ztss_gps_failure.hpp"
#include "ztss_cases/ztss_motor_esc_failure/ztss_motor_esc_failure.hpp"
#include "ztss_cases/ztss_motor_saturation/ztss_motor_saturation.hpp"
#include "ztss_cases/ztss_pose_estimate_lost/ztss_pose_estimate_lost.hpp"
#include "ztss_cases/ztss_vibration/ztss_vibration.hpp"

using namespace time_literals;

class ZtssMonitor : public ModuleBase<ZtssMonitor>, public ModuleParams
{
public:

	ZtssMonitor();
	~ZtssMonitor()=default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ZtssMonitor *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

ZtssCaseAttitudeFailure  ztss_case_attitude_failure_{this};
ZtssCaseEkfDivergence    ztss_case_ekf_divergence_{this};
// Sensor check
ZtssCaseGpsFailure              ztss_case_gps_failure_{this};
ZtssCaseImuFailure              ztss_case_imu_failure_{this};
ZtssCaseMagnetometerFailure     ztss_case_magnetometer_failure_{this};
ZtssCaseBarometerFailure        ztss_case_barometer_failure_{this};
// Actuator check
ZtssCaseMotorEscFailure         ztss_case_motor_esc_failure_{this};
ZtssCaseMotorSaturation          ztss_case_motor_saturation_{this};
// Basic Functionality
ZtssCasePoseEstimateLost ztss_case_pose_estimate_lost_{this};
// Vibration
ZtssCaseVibration        ztss_case_vibration_{this};

// perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};



};
