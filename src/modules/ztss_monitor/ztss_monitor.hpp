#pragma once

#include "worker_thread.hpp"
#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/critical_action.h>

#include "ztss_cases/ztss_case_dummy/ztss_case_dummy.hpp"

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

ZtssCaseDummy  ztss_case_dummy_{this};

// perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};



};
