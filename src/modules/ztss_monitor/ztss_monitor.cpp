#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/defines.h>
#include "ztss_monitor.hpp"

ZtssMonitor::ZtssMonitor() : ModuleParams(nullptr)
{

}


int ZtssMonitor::custom_command(int argc, char* argv[])
{
	return 0;
}


int ZtssMonitor::print_usage(const char * reason)
{
	return 0;
}

int ZtssMonitor::print_status()
{
	return 0;
}

int ZtssMonitor::task_spawn(int argc, char *argv[])
{
	// task_spawn() calls px4_task_spawn_cmd() → spawns the thread
	// New thread starts at run_trampoline() → entry point
	// run_trampoline() calls run() → main loop starts
	_task_id = px4_task_spawn_cmd("ztss_monitor",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 20,
				      PX4_STACK_ADJUSTED(3250), // to be adjusted
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);
	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}
	// wait_until_running() returns → task is fully initialized
	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return 0;
}

ZtssMonitor* ZtssMonitor::instantiate(int argc, char *argv[])
{
	ZtssMonitor* instance = new ZtssMonitor();

	if (instance==nullptr)
	{
		PX4_ERR("Ztss Monitor Allocation Failed");
	}
	return instance;
}


// commander start
//     ↓
// commander_main(argc, argv)
//     ↓
// Commander::main(argc, argv)  [from ModuleBase]
//     ↓
// Commander::task_spawn(argc, argv)
//     ↓
// px4_task_spawn_cmd() → Creates thread
//     ↓
// run_trampoline()
//     ↓
// Commander::run()  ← MAIN LOOP STARTS HERE
void ZtssMonitor::run()
{
	PX4_INFO("Ztss Monitor Module Started");

	while (!should_exit())
	{
		this->ztss_case_dummy_.update_subscribed_values();
		this->ztss_case_dummy_.execute_use_case_safety_evaluation();
		this->ztss_case_dummy_.publish_use_case_status();
		px4_usleep(50_ms); // 20 hz
	}

	PX4_INFO("Ztss Monitor Module Stopped");

}


extern "C" __EXPORT int ztss_monitor_main(int argc, char* argv[]){return ZtssMonitor::main(argc, argv);}
