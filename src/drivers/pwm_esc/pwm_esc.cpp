/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pwm_esc.cpp
 * Driver for the NuttX PWM driver controlled escs
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/sem.hpp>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <fcntl.h>

#include <drivers/drv_pwm_output.h>

#include <lib/mixer_module/mixer_module.hpp>
#include <perf/perf_counter.h>
#include <parameters/param.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>

#include <nuttx/timers/pwm.h>

#ifndef PWMESC_OUT_PATH
#  define PWMESC_OUT_PATH "/dev/pwmX";
#endif

/* Number of PWMESC device nodes /dev/pwm0 .. /dev/pwmX */

#ifndef PWMESC_N_DEVICES
#  define PWMESC_N_DEVICES 1
#  define PWMESC_CHANNELS_PER_DEV {CONFIG_PWM_NCHANNELS}
#  define PWMESC_N_CHANNELS CONFIG_PWM_NCHANNELS
#else
#  ifndef PWMESC_CHANNELS_PER_DEV
#    error "Define the number of PWM channels for each device (PWMESC_CHANNELS_PER_DEV)"
#  endif
#  ifndef PWMESC_N_CHANNELS
#    error "Define the total number of PWM channels (PWMESC_N_CHANNELS)"
#  endif
#endif

#ifndef PWM_DEFAULT_RATE
#  define PWM_DEFAULT_RATE 400
#endif

//using namespace time_literals;

/**
 * The PWMESC class.
 *
 */
class PWMESC : public OutputModuleInterface
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	PWMESC(bool hitl);

	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	virtual ~PWMESC();

	/**
	 * Initialize the PWMESC class.
	 *
	 * Retrieve relevant initial system parameters. Connect to PWM device
	 *
	 * @param hitl_mode set to suppress publication of actuator_outputs
	 */
	int			init(bool hitl_mode);

	/**
	 * Start the PWMESC driver
	 */
	static int		start(int argc, char *argv[]);

	/**
	 * Stop the PWMESC driver
	 */
	static int		stop();

	/**
	 * Status of PWMESC driver
	*/
	static int              status();

	/**
	 * Usage of PWMESC driver
	*/
	static int print_usage(const char *reason);

	/**
	 * Return if the PWMESC driver is already running
	 */
	bool		running() {return _initialized;};

	/**
	 * updateOutputs
	 *
	 * Sets the actual PWM outputs. See OutputModuleInterface
	 *
	 */

	bool		updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				      unsigned num_outputs, unsigned num_control_groups_updated) override;

	/**
	 * Don't allow more channels than MAX_ACTUATORS
	*/
	static_assert(PWMESC_N_CHANNELS <= MAX_ACTUATORS, "Increase MAX_ACTUATORS if this fails");

private:

	bool _initialized{false};

	volatile int		_task;			///< worker task id
	volatile bool		_task_should_exit;	///< worker terminate flag

	px4_sem_t _update_sem;

	perf_counter_t		_perf_update;		///< local performance counter for PWM updates

	/* subscribed topics */

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	MixingOutput _mixing_output;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};

	/* advertised topics */
	uORB::PublicationMulti<actuator_outputs_s>		_actuator_outputs_sim_pub{ORB_ID(actuator_outputs_sim)};

	actuator_armed_s		_actuator_armed;

	bool                    _hitl_mode;     ///< Hardware-in-the-loop simulation mode - don't publish actuator_outputs

	int		_pwm_fd[PWMESC_N_DEVICES];

	int32_t		_pwm_rate{PWM_DEFAULT_RATE};

	int		init_pwm_outputs();

	/* Singleton pointer */
	static PWMESC	*_instance;

	/**
	 * Status of PWMESC driver
	*/
	int                     printStatus();

	/**
	 * Trampoline to the worker task
	 */
	static int		task_main_trampoline(int argc, char *argv[]);

	/**
	 * worker task
	 */
	void			task_main();

	/**
	 * Callback for mixer subscriptions
	 */
	void Run() override;

	void update_params();

	/* No copy constructor */
	PWMESC(const PWMESC &);
	PWMESC operator=(const PWMESC &);

	/**
	 * Get the singleton instance
	 */
	static inline PWMESC *getInstance(bool allocate = false, bool hitl = false)
	{
		if (_instance == nullptr && allocate) {
			/* create the driver */
			_instance = new PWMESC(hitl);
		}

		return _instance;
	}

	/**
	 * Set the PWMs on open device fd to 0 duty cycle
	 * and start or stop (request == PWMIOC_START / PWMIOC_STOP)
	 */
	int set_defaults(int fd, unsigned long request);
};

PWMESC *PWMESC::_instance = nullptr;

PWMESC::PWMESC(bool hitl) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_task(-1),
	_task_should_exit(false),
	_perf_update(perf_alloc(PC_ELAPSED, "pwm update")),
	_mixing_output(PARAM_PREFIX, PWMESC_N_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto,
		       true),
	_hitl_mode(hitl)
{
	/* initialize tick semaphores */
	px4_sem_init(&_update_sem, 0, 0);
	px4_sem_setprotocol(&_update_sem, SEM_PRIO_NONE);

	/* clear armed status */
	memset(&_actuator_armed, 0, sizeof(actuator_armed_s));
}

PWMESC::~PWMESC()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		px4_usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		PX4_ERR("Task exit fail\n");
		px4_task_delete(_task);
	}

	/* deallocate perfs */
	perf_free(_perf_update);

	px4_sem_destroy(&_update_sem);
}

int
PWMESC::init(bool hitl_mode)
{
	/* start the main task */
	_task = px4_task_spawn_cmd("pwm_esc",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				   PX4_STACK_ADJUSTED(3048),
				   (px4_main_t)&PWMESC::task_main_trampoline,
				   nullptr);

	if (_task < 0) {
		PX4_ERR("task start failed: %d", errno);
		return -errno;
	}

	_initialized = true;

	/* schedule workqueue */
	ScheduleNow();

	return OK;
}

int
PWMESC::task_main_trampoline(int argc, char *argv[])
{
	getInstance()->task_main();
	return 0;
}

bool
PWMESC::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		      unsigned num_control_groups_updated)
{
	bool ret = true;
	struct pwm_info_s pwm;
	const unsigned ch_per_dev[PWMESC_N_DEVICES] = PWMESC_CHANNELS_PER_DEV;

	if (!_hitl_mode) {
		/* When not in hitl, run PWMs */
		unsigned n_out = 0;

		for (int dev = 0; dev < PWMESC_N_DEVICES && _pwm_fd[dev] >= 0; dev++) {
			memset(&pwm, 0, sizeof(struct pwm_info_s));
			pwm.frequency = _pwm_rate;

			unsigned i;

			for (i = 0; i < ch_per_dev[dev] && n_out < num_outputs; i++) {
				uint16_t pwm_val = outputs[n_out++];
				pwm.channels[i].duty = ((((uint32_t)pwm_val) << 16) / (1000000 / _pwm_rate));
				pwm.channels[i].channel = i + 1;
			}

			if (i < CONFIG_PWM_NCHANNELS) {
				pwm.channels[i].channel = -1;
			}

			if (::ioctl(_pwm_fd[dev], PWMIOC_SETCHARACTERISTICS,
				    (unsigned long)((uintptr_t)&pwm)) < 0) {
				PX4_ERR("PWMIOC_SETCHARACTERISTICS for pwm%d failed: %d\n", dev,
					errno);
				ret = false;
			}
		}

	} else {
		// In hitl, publish actuator_outputs_sim
		// Only publish once we receive actuator_controls (important for lock-step to work correctly)

		if (num_control_groups_updated > 0) {
			actuator_outputs_s actuator_outputs_sim{};
			actuator_outputs_sim.noutputs = num_outputs;

			const uint32_t reversible_outputs = _mixing_output.reversibleOutputs();

			for (int i = 0; i < (int)num_outputs; i++) {
				uint16_t disarmed = _mixing_output.disarmedValue(i);
				uint16_t min = _mixing_output.minValue(i);
				uint16_t max = _mixing_output.maxValue(i);

				OutputFunction function = _mixing_output.outputFunction(i);
				bool is_reversible = reversible_outputs & (1u << i);
				float output = outputs[i];

				if (((int)function >= (int)OutputFunction::Motor1 && (int)function <= (int)OutputFunction::MotorMax
				     && !is_reversible)) {
					// Scale non-reversible motors to [0, 1]
					actuator_outputs_sim.output[i] = (output - disarmed) / (max - disarmed);

				} else {
					// Scale everything else to [-1, 1]
					const float pwm_center = (max + min) / 2.f;
					const float pwm_delta = (max - min) / 2.f;
					actuator_outputs_sim.output[i] = (output - pwm_center) / pwm_delta;
				}
			}

			actuator_outputs_sim.timestamp = hrt_absolute_time();
			_actuator_outputs_sim_pub.publish(actuator_outputs_sim);
		}

		ret = true;
	}

	return ret;
}

int
PWMESC::set_defaults(int fd, unsigned long request)
{
	/* Configure PWM to default rate, 1 as duty and start */

	int ret = -1;
	const int ch_per_dev[PWMESC_N_DEVICES] = PWMESC_CHANNELS_PER_DEV;
	struct pwm_info_s pwm;

	for (int dev = 0; dev < PWMESC_N_DEVICES; dev++) {
		memset(&pwm, 0, sizeof(struct pwm_info_s));
		pwm.frequency = _pwm_rate;

		int i;

		for (i = 0; i < ch_per_dev[dev]; i++) {
			pwm.channels[i].duty = 1; /* 0 is not allowed duty cycle value */
			pwm.channels[i].channel = i + 1;
		}

		if (i < CONFIG_PWM_NCHANNELS) {
			pwm.channels[i].channel = -1;
		}

		/* Set the frequency and duty */

		ret = ::ioctl(fd, PWMIOC_SETCHARACTERISTICS,
			      (unsigned long)((uintptr_t)&pwm));

		if (ret < 0) {
			PX4_ERR("PWMIOC_SETCHARACTERISTICS failed: %d\n",
				errno);

		} else {

			/* Start / stop */

			ret = ::ioctl(fd, request, 0);

			if (ret < 0) {
				PX4_ERR("PWMIOC_START/STOP failed: %d\n", errno);
			}
		}
	}

	return ret;
}

int
PWMESC::init_pwm_outputs()
{
	int ret = -1;

	_mixing_output.setIgnoreLockdown(_hitl_mode);
	_mixing_output.setMaxNumOutputs(PWMESC_N_CHANNELS);
	const int update_interval_in_us = math::constrain(1000000 / (_pwm_rate * 2), 500, 100000);
	_mixing_output.setMaxTopicUpdateRate(update_interval_in_us);

	/* Open the PWM devnode */

	// loop through the devices, open all fd:s
	char pwm_device_name[] = PWMESC_OUT_PATH;

	for (int i = 0; i < PWMESC_N_DEVICES; i++) {
		pwm_device_name[sizeof(pwm_device_name) - 2] = '0' + i;

		_pwm_fd[i] = ::open(pwm_device_name, O_RDONLY);

		if (_pwm_fd[i] >= 0) {
			/* Configure PWM to default rate, 0 pulse and start */

			ret = set_defaults(_pwm_fd[i], PWMIOC_START);

			if (ret != 0) {
				PX4_ERR("Init failed for %s errno %d", pwm_device_name, errno);
				break;
			}
		}
	}

	if (ret != 0) {
		PX4_ERR("PWM init failed");
	}

	return ret;
}

void
PWMESC::Run()
{
	/* Just trigger the main task */
	px4_sem_post(&_update_sem);
}

void
PWMESC::task_main()
{
	if (init_pwm_outputs() != 0) {
		PX4_ERR("PWM initialization failed");
		_task_should_exit = true;
	}

	while (!_task_should_exit) {

		/* Get the armed status */

		_actuator_armed_sub.update(&_actuator_armed);

		struct timespec ts;
		px4_clock_gettime(CLOCK_REALTIME, &ts);
		/* Add 100 ms, this can't overflow */
		ts.tv_nsec += 100000000;

		if (ts.tv_nsec >= 1000000000) {
			ts.tv_nsec -= 1000000000;
			ts.tv_sec += 1;
		}

		/* sleep waiting for mixer update */
		int ret = px4_sem_timedwait(&_update_sem, &ts);

		perf_begin(_perf_update);

		if (ret == 0) {
			_mixing_output.update();
		}

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			update_params();
		}

		_mixing_output.updateSubscriptions(true);

		perf_end(_perf_update);
	}

	PX4_DEBUG("exiting");

	/* Configure PWM to default rate, 0 pulse and stop */

	for (int i = 0; i < PWMESC_N_DEVICES; i++) {
		if (_pwm_fd[i] >= 0) {
			set_defaults(_pwm_fd[i], PWMIOC_STOP);
		}
	}

	/* tell the dtor that we are exiting */
	_task = -1;
}

void PWMESC::update_params()
{
	/* skip update when armed */
	if (_actuator_armed.armed) {
		return;
	}

	/* Call MixingOutput::updateParams */
	updateParams();
}

extern "C" __EXPORT int pwm_esc_main(int argc, char *argv[]);

int
PWMESC::start(int argc, char *argv[])
{
	int ret = 0;
	int32_t hitl_mode;

	if (param_get(param_find("SYS_HITL"), &hitl_mode) != PX4_OK) {
		PX4_ERR("Can't read parameter SYS_HITL");
		return -1;
	}

	if (PWMESC::getInstance(true, hitl_mode != 0) == nullptr) {
		PX4_ERR("Driver allocation failed");
		return -1;
	}

	if (PWMESC::getInstance()->running()) {
		PX4_ERR("Already running");
		return -1;
	}

	if (OK != PWMESC::getInstance()->init(hitl_mode)) {
		delete PWMESC::getInstance();
		PX4_ERR("Driver init failed");
		return -1;
	}

	return ret;
}

int
PWMESC::stop()
{
	if (PWMESC::getInstance() == nullptr) {
		PX4_ERR("Not started");
		return -1;
	}

	if (!PWMESC::getInstance()->running()) {
		PX4_ERR("Not running");
		return -1;
	}

	delete (PWMESC::getInstance());
	_instance = nullptr;

	return 0;
}

int PWMESC::printStatus()
{
	_mixing_output.printStatus();
	return 0;
}

int
PWMESC::status()
{
	if (PWMESC::getInstance() == nullptr) {
		PX4_INFO("Not started");
		return 0;
	}

	if (!PWMESC::getInstance()->running()) {
		PX4_ERR("Not running");
		return -1;
	}

	return PWMESC::getInstance()->printStatus();
}

int PWMESC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for PWM outputs. Used also in HITL mode.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_esc", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the module");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int
pwm_esc_main(int argc, char *argv[])
{
	/* check for sufficient number of arguments */
	if (argc < 2) {
		PWMESC::print_usage("Need a command");
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		return PWMESC::start(argc - 1, argv + 1);
	} else if (!strcmp(argv[1], "stop")) {
		return PWMESC::stop();
	} else if (!strcmp(argv[1], "status")) {
		return PWMESC::status();
	} else {
		PWMESC::print_usage("Invalid command");
	}

	return -1;
}
