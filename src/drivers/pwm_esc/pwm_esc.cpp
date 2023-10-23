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
 * Driver for the NuttX PWM driver controleed escs
 *
 */

#include <px4_platform_common/px4_config.h>
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

#include <drivers/device/device.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <lib/mixer_module/mixer_module.hpp>
#include <perf/perf_counter.h>
#include <parameters/param.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

#include <nuttx/timers/pwm.h>

/* Note: This driver will register two device nodes: /dev/px4fmu and
 * /dev/pwm_output[0|1] depending on whether px4io has been loaded previously
 */

#define PX4FMU_DEVICE_PATH "/dev/px4fmu"

#ifndef PWMESC_OUT_PATH
#define PWMESC_OUT_PATH "/dev/pwmX";
#endif

#ifndef PWMESC_MAX_DEVICES
#define PWMESC_MAX_DEVICES 2
#endif

#ifndef PWMESC_MAX_CHANNELS
#define PWMESC_MAX_CHANNELS CONFIG_PWM_NCHANNELS
#endif

#ifndef PWM_DEFAULT_RATE
#define PWM_DEFAULT_RATE 400
#endif

//using namespace time_literals;

/**
 * The PWMESC class.
 *
 */
class PWMESC : public cdev::CDev, public OutputModuleInterface
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	PWMESC();

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
	int                     printStatus();
	static int              status();

	/**
	 * Return if the PWMESC driver is already running
	 */
	bool		running() {return _initialized;};

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the PWMESC file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

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
	static_assert(PWMESC_MAX_CHANNELS <= MAX_ACTUATORS, "Increase MAX_ACTUATORS if this fails");

private:

	bool _initialized{false};

	volatile int		_task;			///< worker task id
	volatile bool		_task_should_exit;	///< worker terminate flag

	px4_sem_t _update_sem;

	perf_counter_t		_perf_update;		///< local performance counter for PWM updates

	/* subscribed topics */

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	MixingOutput _mixing_output{PARAM_PREFIX, PWMESC_MAX_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	bool _pwm_min_configured{false};
	bool _pwm_max_configured{false};
	bool _pwm_fail_configured{false};
	bool _pwm_dis_configured{false};
	bool _pwm_rev_configured{false};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};

	/* advertised topics */
	uORB::PublicationMulti<actuator_outputs_s>		_actuator_outputs_pub{ORB_ID(actuator_outputs)};

	actuator_armed_s		_actuator_armed;

	bool                    _hitl_mode;     ///< Hardware-in-the-loop simulation mode - don't publish actuator_outputs

	int		_pwm_fd[PWMESC_MAX_DEVICES];

	int32_t		_pwm_disarmed_default;
	int32_t		_pwm_rate{PWM_DEFAULT_RATE};
	int		_class_instance{-1};
	bool		_force_safety{false};
	int		init_pwm_outputs();

	/* Singleton pointer */
	static PWMESC	*_instance;

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
	static inline PWMESC *getInstance()
	{
		if (_instance == nullptr) {
			/* create the driver */
			_instance = new PWMESC();
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

PWMESC::PWMESC() :
	CDev(PX4FMU_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_task(-1),
	_task_should_exit(false),
	_perf_update(perf_alloc(PC_ELAPSED, "pwm update")),
	_hitl_mode(false)
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

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	/* deallocate perfs */
	perf_free(_perf_update);

	px4_sem_destroy(&_update_sem);
}

int
PWMESC::init(bool hitl_mode)
{
	int ret;

	_hitl_mode = hitl_mode;

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* Claim the generic PWM output device node */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	/* start the main task */
	_task = px4_task_spawn_cmd("pwm_esc",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				   3048,
				   (px4_main_t)&PWMESC::task_main_trampoline,
				   nullptr);

	if (_task < 0) {
		PX4_ERR("task start failed: %d", errno);
		return -errno;
	}

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

	memset(&pwm, 0, sizeof(struct pwm_info_s));
	pwm.frequency = _pwm_rate;

	for (unsigned i = 0; i < num_outputs; i++) {
		// TODO: channel to proper pwm device map.
		// this is now just quick hack for one pwm device, direct map of channels

		uint16_t pwm_val = _force_safety ? _mixing_output.disarmedValue(i) : outputs[i];
		pwm.channels[i].duty = ((((uint32_t)pwm_val) << 16) / (1000000 / _pwm_rate));
		pwm.channels[i].channel = i + 1;
	}

	/* Only publish if not in hitl */

	if (!_hitl_mode &&
	    ::ioctl(_pwm_fd[0], PWMIOC_SETCHARACTERISTICS,
		    (unsigned long)((uintptr_t)&pwm)) < 0) {
		PX4_ERR("PWMIOC_SETCHARACTERISTICS failed: %d\n",
			errno);
		ret = false;
	}

	return ret;
}

int
PWMESC::set_defaults(int fd, unsigned long request)
{
	/* Configure PWM to default rate, 1 as duty and start */

	struct pwm_info_s pwm;
	memset(&pwm, 0, sizeof(struct pwm_info_s));
	pwm.frequency = _pwm_rate;

	for (int j = 0; j < PWMESC_MAX_CHANNELS; j++) {
		pwm.channels[j].duty = 1; /* 0 is not allowed duty cycle value */
		pwm.channels[j].channel = j + 1;
	}

	/* Set the frequency and duty */

	int ret = ::ioctl(fd, PWMIOC_SETCHARACTERISTICS,
			  (unsigned long)((uintptr_t)&pwm));

	if (ret < 0) {
		PX4_ERR("PWMIOC_SETCHARACTERISTICS) failed: %d\n",
			errno);

	} else {

		/* Start / stop */

		ret = ::ioctl(fd, request, 0);

		if (ret < 0) {
			PX4_ERR("PWMIOC_START/STOP failed: %d\n", errno);
		}

	}

	return ret;
}

int
PWMESC::init_pwm_outputs()
{
	int ret = 0;

	update_params();

	_mixing_output.setIgnoreLockdown(_hitl_mode);
	_mixing_output.setMaxNumOutputs(PWMESC_MAX_CHANNELS);
	const int update_interval_in_us = math::constrain(1000000 / (_pwm_rate * 2), 500, 100000);
	_mixing_output.setMaxTopicUpdateRate(update_interval_in_us);

	/* Open the PWM devnode */

	// TODO: loop through the devices, open all fd:s
	char pwm_device_name[] = PWMESC_OUT_PATH;
	int n_pwm_devices = 1;

	for (int i = 0; i < 1 && i < n_pwm_devices; i++) {
		pwm_device_name[sizeof(pwm_device_name) - 2] = '0' + i;

		_pwm_fd[i] = ::open(pwm_device_name, O_RDONLY);

		if (_pwm_fd[i] < 0) {
			PX4_ERR("pwm_main: open %s failed: %d\n", pwm_device_name, errno);
			ret = -1;
			continue;
		}

		/* Configure PWM to default rate, 0 pulse and start */

		if (set_defaults(_pwm_fd[i], PWMIOC_START) != 0) {
			ret = -1;
		}
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
	while (!_initialized) {
		usleep(10000);
	}

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
		}

		_mixing_output.updateSubscriptions(true);

		perf_end(_perf_update);
	}

	PX4_DEBUG("exiting");

	/* Configure PWM to default rate, 0 pulse and stop */

	int n_pwm_devices = 1;

	for (int i = 0; i < 1 && i < n_pwm_devices; i++) {
		set_defaults(_pwm_fd[i], PWMIOC_STOP);
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

	int32_t _pwm_min_default = PWM_DEFAULT_MIN;
	int32_t _pwm_max_default = PWM_DEFAULT_MAX;
	_pwm_disarmed_default = PWM_MOTOR_OFF;

	uint32_t pwm_default_channel_mask = 0;
	char str[18];
	const char *prefix;

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		param_get(param_find("PWM_MAIN_MIN"), &_pwm_min_default);
		param_get(param_find("PWM_MAIN_MAX"), &_pwm_max_default);
		param_get(param_find("PWM_MAIN_DISARM"), &_pwm_disarmed_default);
		param_get(param_find("PWM_MAIN_RATE"), &_pwm_rate);
		prefix = "PWM_MAIN";

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		param_get(param_find("PWM_AUX_MIN"), &_pwm_min_default);
		param_get(param_find("PWM_AUX_MAX"), &_pwm_max_default);
		param_get(param_find("PWM_AUX_DISARM"), &_pwm_disarmed_default);
		param_get(param_find("PWM_AUX_RATE"), &_pwm_rate);
		prefix = "PWM_AUX";

	} else if (_class_instance == CLASS_DEVICE_TERTIARY) {
		param_get(param_find("PWM_EXTRA_MIN"), &_pwm_min_default);
		param_get(param_find("PWM_EXTRA_MAX"), &_pwm_max_default);
		param_get(param_find("PWM_EXTRA_DISARM"), &_pwm_disarmed_default);
		param_get(param_find("PWM_EXTRA_RATE"), &_pwm_rate);
		prefix = "PWM_EXTRA";

	} else {
		PX4_ERR("invalid class instance %d", _class_instance);
		return;
	}

	/* Init to a known state - never a bad idea although might look unnecessary */
	for (unsigned i = 0; i <  PWMESC_MAX_CHANNELS; i++) {
		if (!_pwm_fail_configured) {
			_mixing_output.failsafeValue(i) = 0;
		}

		if (!_pwm_dis_configured) {
			_mixing_output.disarmedValue(i) = _pwm_disarmed_default;
		}

		if (!_pwm_min_configured) {
			_mixing_output.minValue(i) = _pwm_min_default;
		}

		if (!_pwm_max_configured) {
			_mixing_output.maxValue(i) = _pwm_max_default;
		}
	}

	// PWM_*_MINx (PWM_MAIN_MIN, PWM_AUX_MIN or PWM_EXTRA_MIN)
	if (!_pwm_min_configured) {
		for (unsigned i = 0; i < PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_MIN%u", prefix, i + 1);
			int32_t pwm_min = -1;

			if (param_get(param_find(str), &pwm_min) == PX4_OK) {
				if (pwm_min >= 0 && pwm_min != 1000) {
					_mixing_output.minValue(i) = math::constrain(pwm_min, static_cast<int32_t>(PWM_LOWEST_MIN),
								     static_cast<int32_t>(PWM_HIGHEST_MIN));

					if (pwm_min != _mixing_output.minValue(i)) {
						int32_t pwm_min_new = _mixing_output.minValue(i);
						param_set(param_find(str), &pwm_min_new);
					}

				} else if (pwm_default_channel_mask & 1 << i) {
					_mixing_output.minValue(i) = _pwm_min_default;
				}
			}
		}

		_pwm_min_configured = true;
	}

	// PWM_*_MAXx
	if (!_pwm_max_configured) {
		for (unsigned i = 0; i < PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_MAX%u", prefix, i + 1);
			int32_t pwm_max = -1;

			if (param_get(param_find(str), &pwm_max) == PX4_OK) {
				if (pwm_max >= 0 && pwm_max != 2000) {
					_mixing_output.maxValue(i) = math::constrain(pwm_max, static_cast<int32_t>(PWM_LOWEST_MAX),
								     static_cast<int32_t>(PWM_HIGHEST_MAX));

					if (pwm_max != _mixing_output.maxValue(i)) {
						int32_t pwm_max_new = _mixing_output.maxValue(i);
						param_set(param_find(str), &pwm_max_new);
					}

				} else if (pwm_default_channel_mask & 1 << i) {
					_mixing_output.maxValue(i) = _pwm_max_default;
				}
			}
		}

		_pwm_max_configured = true;
	}

	// PWM_*_FAILx
	if (!_pwm_fail_configured) {
		for (unsigned i = 0; i < PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_FAIL%u", prefix, i + 1);
			int32_t pwm_fail = -1;

			if (param_get(param_find(str), &pwm_fail) == PX4_OK) {
				if (pwm_fail >= 0) {
					_mixing_output.failsafeValue(i) = math::constrain(pwm_fail, static_cast<int32_t>(0),
									  static_cast<int32_t>(PWM_HIGHEST_MAX));

					if (pwm_fail != _mixing_output.failsafeValue(i)) {
						int32_t pwm_fail_new = _mixing_output.failsafeValue(i);
						param_set(param_find(str), &pwm_fail_new);
					}
				}
			}
		}

		_pwm_fail_configured = true;
	}

	// PWM_*_DISx
	if (!_pwm_dis_configured) {
		for (unsigned i = 0; i < PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_DIS%u", prefix, i + 1);
			int32_t pwm_dis = -1;

			if (param_get(param_find(str), &pwm_dis) == PX4_OK) {
				if (pwm_dis >= 0 && pwm_dis != 900) {
					_mixing_output.disarmedValue(i) = math::constrain(pwm_dis, static_cast<int32_t>(0),
									  static_cast<int32_t>(PWM_HIGHEST_MAX));

					if (pwm_dis != _mixing_output.disarmedValue(i)) {
						int32_t pwm_dis_new = _mixing_output.disarmedValue(i);
						param_set(param_find(str), &pwm_dis_new);
					}

				} else if (pwm_default_channel_mask & 1 << i) {
					_mixing_output.disarmedValue(i) = _pwm_disarmed_default;
				}
			}
		}

		_pwm_dis_configured = true;
	}

	// PWM_*_REVx
	if (!_pwm_rev_configured) {
		uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
		reverse_pwm_mask = 0;

		for (unsigned i = 0; i < PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_REV%u", prefix, i + 1);
			int32_t pwm_rev = -1;

			if (param_get(param_find(str), &pwm_rev) == PX4_OK) {
				if (pwm_rev >= 1) {
					reverse_pwm_mask |= (1 << i);

				} else {
					reverse_pwm_mask = reverse_pwm_mask & ~(1 << i);
				}

			}
		}

		_pwm_rev_configured = true;
	}

	// PWM_*_TRIMx
	{
		int16_t values[8] {};

		for (unsigned i = 0; i < PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_TRIM%u", prefix, i + 1);
			float pwm_trim = 0.f;

			if (param_get(param_find(str), &pwm_trim) == PX4_OK) {
				values[i] = roundf(10000 * pwm_trim);
			}
		}

		if (_mixing_output.mixers()) {
			// copy the trim values to the mixer offsets
			_mixing_output.mixers()->set_trims(values, PWMESC_MAX_CHANNELS);
		}
	}
}

int
PWMESC::ioctl(file *filep, int cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {
	case PWM_SERVO_ARM:
		/* set the 'armed' bit */
		break;

	case PWM_SERVO_SET_ARM_OK:
		/* set the 'OK to arm' bit */
		break;

	case PWM_SERVO_CLEAR_ARM_OK:
		/* clear the 'OK to arm' bit */
		break;

	case PWM_SERVO_DISARM:
		/* clear the 'armed' bit */
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		/* get the default update rate */
		*(unsigned *)arg = PWM_DEFAULT_RATE;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		/* set the update rate */
		_pwm_rate = arg;
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		/* get the alternative update rate */
		*(unsigned *)arg = _pwm_rate;
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(unsigned *)arg = 0;
		break;

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.failsafeValue(i);
			}

			pwm->channel_count = MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM:  {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.disarmedValue(i);
			}

			pwm->channel_count = MAX_ACTUATORS;
			break;
		}

		break;

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (i < MAX_ACTUATORS) {
					_mixing_output.minValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = MAX_ACTUATORS;

			for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.minValue(i);
			}

			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (i < MAX_ACTUATORS) {
					_mixing_output.maxValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = MAX_ACTUATORS;

			for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.maxValue(i);
			}

			break;
		}

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = PWMESC_MAX_CHANNELS;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		_force_safety = false;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		_force_safety = true;
		break;

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET(0);

			if (channel >= MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			char pwm_device_name[] = PWMESC_OUT_PATH;
			pwm_device_name[sizeof(pwm_device_name) - 2] = '0';

			int fd = ::open(pwm_device_name, O_RDONLY);

			if (fd < 0) {
				ret = -EINVAL;

			} else {

				/* fetch a current PWM value */
				pwm_info_s pwm;
				ret = ::ioctl(fd, PWMIOC_GETCHARACTERISTICS,
					      (unsigned long)((uintptr_t)&pwm));

				/* calculate duty cycle in us and round properly */
				uint32_t duty = pwm.channels[channel].duty * (1000000 / _pwm_rate);
				*(servo_position_t *)arg = (duty >> 16) + ((duty & 0x8000) >> 15);

				::close(fd);
			}

			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {
			unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

			if (channel >= MAX_ACTUATORS) {
				ret = -EINVAL;

			} else {
				/* rategroups not supported, it is always 0 */
				*(uint32_t *)arg = 0;
			}

			break;
		}

	case MIXERIOCRESET:
		/* Can start the main thread now */
		_initialized = true;
		_mixing_output.resetMixer();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixer(buf, buflen);
			break;
		}

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filep, cmd, arg);
		break;
	}

	return ret;
}

extern "C" __EXPORT int pwm_esc_main(int argc, char *argv[]);

int
PWMESC::start(int argc, char *argv[])
{
	int ret = 0;

	if (PWMESC::getInstance() == nullptr) {
		PX4_ERR("Driver allocation failed");
		return -1;
	}

	if (PWMESC::getInstance()->running()) {
		PX4_ERR("Already running");
		return -1;
	}

	bool hitl_mode = false;

	/* Check if started in hil mode */
	for (int extra_args = 1; extra_args < argc; extra_args++) {
		if (!strcmp(argv[extra_args], "hil")) {
			hitl_mode = true;

		} else if (argv[extra_args][0] != '\0') {
			PX4_WARN("unknown argument: %s", argv[extra_args]);
		}
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
		PX4_ERR("Driver allocation failed");
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
		PX4_ERR("status: Driver allocation failed");
		return -1;
	}

	if (!PWMESC::getInstance()->running()) {
		PX4_ERR("status: Not running");
		return -1;
	}

	PWMESC::getInstance()->printStatus();
	return 0;
}


int
pwm_esc_main(int argc, char *argv[])
{
	/* check for sufficient number of arguments */
	if (argc < 2) {
		PX4_ERR("Need a command, try 'start' / 'stop' / 'status'");
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		return PWMESC::start(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "stop")) {
		return PWMESC::stop();
	}

	if (!strcmp(argv[1], "status")) {
		return PWMESC::status();
	}


	return 0;
}
