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
#include <sys/ioctl.h>
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
#include <uORB/topics/esc_status.h>

#ifdef CONFIG_MODULES_REDUNDANCY
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/redundancy_status.h>

#define MAX_N_FCS vehicle_status_s::MAX_REDUNDANT_CONTROLLERS
#endif

#include <nuttx/timers/pwm.h>
#include <nuttx/timers/dshot.h>

#if defined(CONFIG_DSHOT_NCHANNELS)
#define DSHOT_MAX_CHANNELS CONFIG_DSHOT_NCHANNELS
#else
#define DSHOT_MAX_CHANNELS 8
#endif

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

#define PROTOCOL_PWM_MIN         50
#define PROTOCOL_DSHOT150        -5
#define PROTOCOL_DSHOT300        -4
#define PROTOCOL_DSHOT600        -3
#define PROTOCOL_DSHOT1200       -2

#ifndef PWMESC_DSHOT_PATH
#  define PWMESC_DSHOT_PATH "/dev/dshot0"
#endif

#ifndef PWMESC_DSHOT_N_CHANNELS
#  define PWMESC_DSHOT_N_CHANNELS 4
#endif

#ifndef PWMESC_DSHOT_N_TIMER_GROUPS
#  define PWMESC_DSHOT_N_TIMER_GROUPS 2
#endif

using namespace time_literals;

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
	uORB::PublicationMulti<esc_status_s>			_esc_status_pub{ORB_ID(esc_status)};
	esc_status_s					_esc_status{};

	actuator_armed_s		_actuator_armed;

	bool                    _hitl_mode;     ///< Hardware-in-the-loop simulation mode - don't publish actuator_outputs

	int		_pwm_fd[PWMESC_N_DEVICES];
	uint32_t	_pwm_output_mask{(1u << PWMESC_N_CHANNELS) - 1};

	int32_t		_pwm_rate{PWM_DEFAULT_RATE};

#ifdef CONFIG_DSHOT
	int		_dshot_fd {-1};
	bool		_dshot_enabled{false};
	bool		_dshot_armed{false};
	uint32_t	_dshot_speed{DSHOT_SPEED_600};
	uint32_t	_dshot_output_mask{0};
	uint8_t		_dshot_nchannels{0};
	bool		_dshot_bidir_enabled{false};
#endif

#ifdef CONFIG_MODULES_REDUNDANCY
	uORB::Subscription _redundancy_status_sub {ORB_ID(redundancy_status)};

	uORB::Subscription *_redundant_actuator_outputs_sub[MAX_N_FCS] = {nullptr, nullptr};

	bool _redundant_actuator_control_enabled{false};
#endif

	int		init_pwm_outputs();
	void		update_protocol_config();
	void		publish_dshot_telemetry();
	uint32_t	all_output_mask() const { return (1u << PWMESC_N_CHANNELS) - 1; }

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
	int set_defaults_for_device(int dev, int fd, unsigned long request);
};

PWMESC *PWMESC::_instance = nullptr;

PWMESC::PWMESC(bool hitl) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_task(-1),
	_task_should_exit(false),
	_perf_update(perf_alloc(PC_ELAPSED, "pwm update")),
	_mixing_output(hitl || !PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO) ? "PWM_MAIN" : "PWM_AUX",
		       PWMESC_N_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, true),
	_hitl_mode(hitl)
{
	/* initialize tick semaphores */
	px4_sem_init(&_update_sem, 0, 0);
	px4_sem_setprotocol(&_update_sem, SEM_PRIO_NONE);

	/* clear armed status */
	memset(&_actuator_armed, 0, sizeof(actuator_armed_s));

	for (int i = 0; i < PWMESC_N_DEVICES; ++i) {
		_pwm_fd[i] = -1;
	}

#ifdef CONFIG_DSHOT
#endif
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

#ifdef CONFIG_MODULES_REDUNDANCY
	/* delete redundant_actuator_output subscriptions */

	for (int i = 0; i < MAX_N_FCS; i++) {
		delete _redundant_actuator_outputs_sub[i];
	}

#endif

	px4_sem_destroy(&_update_sem);
}

int
PWMESC::init(bool hitl_mode)
{
#ifdef CONFIG_MODULES_REDUNDANCY
	int32_t spare_autopilots;

	if (param_get(param_find("FT_N_SPARE_FCS"), &spare_autopilots) == PX4_OK && spare_autopilots > 0
	    && spare_autopilots < MAX_N_FCS) {
		/* Find out which actuator_outputs instance we are publishing.
		 * Note: there is a race condition in here in theory; if
		 * drivers publishing actuator outputs are started in parallel
		 * threads, this might give wrong results. However, in practice
		 * all the actuator drivers are started at boot in a single
		 * thread sequentially.
		 */

		int actuator_output_instance = orb_group_count(ORB_ID(actuator_outputs)) - 1;

		/* Sanity check; this only supports 4 instances, since the current
		 * redundancy communication interface (mavlink) doesn't support
		 * sharing more
		 */

		if (actuator_output_instance < 0 || actuator_output_instance > 3) {
			return PX4_ERROR;
		}

		/* Subscribe to the correct actuator outputs from the other FCs;
		 * this assumes that the same actuator control drivers are running
		 * on all FCs; that is, the topic instance numbers match.
		 * There is typically one instance for UAVCAN and another for PWM_ESC.
		 */

		for (int i = 0; i < MAX_N_FCS; i++) {
			const orb_metadata *meta;

			switch (i + redundancy_status_s::FC1) {
			case redundancy_status_s::FC1:
				meta = ORB_ID(redundant_actuator_outputs0);
				break;

			case redundancy_status_s::FC2:
				meta = ORB_ID(redundant_actuator_outputs1);
				break;

			case redundancy_status_s::FC3:
				meta = ORB_ID(redundant_actuator_outputs2);
				break;

			case redundancy_status_s::FC4:
				meta = ORB_ID(redundant_actuator_outputs3);
				break;

			default:
				meta = nullptr;
			}

			_redundant_actuator_outputs_sub[i] = new uORB::Subscription(meta, actuator_output_instance);

			if (!_redundant_actuator_outputs_sub[i]) {
				PX4_ERR("redundant_actuator_outputs%d not available\n", i);
				return PX4_ERROR;
			}
		}

		_redundant_actuator_control_enabled = true;
	}

#endif

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

#ifdef CONFIG_MODULES_REDUNDANCY

	/* If it is known that this FC is not in control of the acutuators, and that there is
	 * actuator output available from the controlling FC, output the controlling FC's
	 * controls instead. This mitigates possible issues with HW or PWM switching logic.
	 * Note that there is an additional delay when the actuator outputs are first via the
	 * communication channel, so we simply assume that the communications rate is high
	 * enough here.
	 *
	 * As a sanity check, verify that the redundancy status and any redundant actuator
	 * outputs are less than 50 ms old.
	 */

	if (_redundant_actuator_control_enabled) {
		redundancy_status_s rstatus;

		if (_redundancy_status_sub.copy(&rstatus) &&
		    hrt_elapsed_time(&rstatus.timestamp) < 50_ms &&
		    rstatus.fc_number != rstatus.fc_in_act_control &&
		    rstatus.fc_number >= redundancy_status_s::FC1 && rstatus.fc_number < redundancy_status_s::FC1 + MAX_N_FCS &&
		    rstatus.fc_in_act_control >= redundancy_status_s::FC1
		    && rstatus.fc_in_act_control < redundancy_status_s::FC1 + MAX_N_FCS) {
			actuator_outputs_s ract_outputs;
			int fc_idx = rstatus.fc_in_act_control - redundancy_status_s::FC1;

			if (_redundant_actuator_outputs_sub[fc_idx]->copy(&ract_outputs) &&
			    hrt_elapsed_time(&ract_outputs.timestamp) < 50_ms) {
				/* The actuator outputs is already in correct scale, simply convert from float to uint16 */

				for (int i = 0; i < (int)num_outputs; i++) {
					outputs[i] = (uint16_t)ract_outputs.output[i];
				}
			}
		}
	}

#endif

	if (!_hitl_mode) {
		/* When not in hitl, run PWMs */
		unsigned channel_offset = 0;

		for (int dev = 0; dev < PWMESC_N_DEVICES && _pwm_fd[dev] >= 0; dev++) {
			memset(&pwm, 0, sizeof(struct pwm_info_s));
			pwm.frequency = _pwm_rate;

			unsigned pwm_index = 0;

			for (unsigned ch = 0; ch < ch_per_dev[dev] && (channel_offset + ch) < num_outputs; ch++) {
				const unsigned output_index = channel_offset + ch;

				if ((_pwm_output_mask & (1u << output_index)) == 0) {
					continue;
				}

				uint16_t pwm_val = outputs[output_index];
				pwm.channels[pwm_index].duty = ((((uint32_t)pwm_val) << 16) / (1000000 / _pwm_rate));
				pwm.channels[pwm_index].channel = ch + 1;
				pwm_index++;
			}

			if (pwm_index > 0) {
				if (pwm_index < CONFIG_PWM_NCHANNELS) {
					pwm.channels[pwm_index].channel = -1;
				}

				if (::ioctl(_pwm_fd[dev], PWMIOC_SETCHARACTERISTICS,
					    (unsigned long)((uintptr_t)&pwm)) < 0) {
					PX4_ERR("PWMIOC_SETCHARACTERISTICS for pwm%d failed: %d\n", dev,
						errno);
					ret = false;
				}
			}

			channel_offset += ch_per_dev[dev];
		}

#ifdef CONFIG_DSHOT

		if (_dshot_enabled && _dshot_fd >= 0 && _dshot_nchannels > 0 && num_outputs >= _dshot_nchannels) {
			const uint8_t dshot_nchannels = math::min(_dshot_nchannels, (uint8_t)DSHOT_MAX_CHANNELS);
			struct dshot_throttle_s dshot_req {};

			dshot_req.ch_mask = (1 << _dshot_nchannels) - 1;

			for (unsigned i = 0; i < dshot_nchannels; ++i) {
				const uint16_t pwm_val = outputs[i];
				const uint16_t disarmed = _mixing_output.disarmedValue(i);
				const uint16_t min = _mixing_output.minValue(i);
				const uint16_t max = _mixing_output.maxValue(i);

				if (stop_motors || pwm_val <= disarmed || max <= min) {
					dshot_req.throttle[i] = 0;

				} else {
					const uint32_t throttle = 48u + (uint32_t)(pwm_val - min) * (2047u - 48u) / (max - min);
					dshot_req.throttle[i] = math::constrain((int)throttle, 48, 2047);
				}

				/* Request telemetry from every channel */

				dshot_req.telemetry_req |= _dshot_bidir_enabled ? 1 << i : 0;
			}

			if (::ioctl(_dshot_fd, DSHOTIOC_SET_THROTTLE, (unsigned long)((uintptr_t)&dshot_req)) < 0) {
				PX4_ERR("DShot update failed: %d", errno);
				ret = false;

			} else {
				publish_dshot_telemetry();
			}
		}

#endif

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
	for (int dev = 0; dev < PWMESC_N_DEVICES; dev++) {
		if (_pwm_fd[dev] == fd) {
			return set_defaults_for_device(dev, fd, request);
		}
	}

	return -EINVAL;
}

int
PWMESC::set_defaults_for_device(int dev, int fd, unsigned long request)
{
	/* Configure PWM to default rate, 1 as duty and start */

	int ret = -1;
	const unsigned ch_per_dev[PWMESC_N_DEVICES] = PWMESC_CHANNELS_PER_DEV;
	struct pwm_info_s pwm;
	unsigned channel_offset = 0;

	for (int i = 0; i < dev; ++i) {
		channel_offset += ch_per_dev[i];
	}

	memset(&pwm, 0, sizeof(struct pwm_info_s));
	pwm.frequency = _pwm_rate;

	unsigned pwm_index = 0;

	for (unsigned ch = 0; ch < ch_per_dev[dev] && (channel_offset + ch) < PWMESC_N_CHANNELS; ch++) {
		if ((_pwm_output_mask & (1u << (channel_offset + ch))) == 0) {
			continue;
		}

		pwm.channels[pwm_index].duty = 1; /* 0 is not allowed duty cycle value */
		pwm.channels[pwm_index].channel = ch + 1;
		pwm_index++;
	}

	if (pwm_index == 0) {
		return OK;
	}

	if (pwm_index < CONFIG_PWM_NCHANNELS) {
		pwm.channels[pwm_index].channel = -1;
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

	return ret;
}

void
PWMESC::publish_dshot_telemetry()
{
#ifdef CONFIG_DSHOT
	static constexpr hrt_abstime ESC_STATUS_TIMEOUT = 20_ms;

	if (!_dshot_enabled || !_dshot_bidir_enabled || _dshot_fd < 0) {
		return;
	}

	struct dshot_telemetry_s telemetry {};

	if (::ioctl(_dshot_fd, DSHOTIOC_GET_TELEMETRY, (unsigned long)((uintptr_t)&telemetry)) < 0) {
		return;
	}

	const uint8_t max_channels = math::min(_dshot_nchannels, esc_status_s::CONNECTED_ESC_MAX);

	_esc_status.timestamp = hrt_absolute_time();
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
	_esc_status.esc_count = math::max(_esc_status.esc_count, max_channels);
	++_esc_status.counter;

	for (uint8_t i = 0; i < max_channels; ++i) {
		const uint8_t mask = 1u << i;

		_esc_status.esc_online_flags |= mask;

		if (_dshot_armed) {
			_esc_status.esc_armed_flags |= mask;

		} else {
			_esc_status.esc_armed_flags &= ~mask;
		}

		_esc_status.esc[i].timestamp = ts_to_abstime(&telemetry.timestamp[i]);
		_esc_status.esc[i].actuator_function = (uint16_t)_mixing_output.outputFunction(i);
		_esc_status.esc[i].esc_rpm = telemetry.erpm[i];

		if (hrt_elapsed_time(&_esc_status.esc[i].timestamp) > ESC_STATUS_TIMEOUT) {
			_esc_status.esc_online_flags &= ~mask;
			_esc_status.esc_armed_flags &= ~mask;
		}
	}

	_esc_status_pub.publish(_esc_status);
#endif
}

void
PWMESC::update_protocol_config()
{
	_pwm_output_mask = all_output_mask();

#ifdef CONFIG_DSHOT
	int32_t tim_config[4] {};
	int32_t dshot_mode = 0;
	bool dshot_requested = false;
	bool dshot_bidir = false;
	param_t timing_param_handles[4] {PARAM_INVALID, PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
	param_t dshot_bidir_param_handles[PWMESC_DSHOT_N_TIMER_GROUPS] {PARAM_INVALID, PARAM_INVALID};
	bool supported_group_selected[PWMESC_DSHOT_N_TIMER_GROUPS] {};
	uint8_t dshot_nchannels = 0;
	constexpr int dshot_group_nchannels = PWMESC_DSHOT_N_CHANNELS / PWMESC_DSHOT_N_TIMER_GROUPS;
	char param_name[17] {};

	for (int timer = 0; timer < 4; ++timer) {
		snprintf(param_name, sizeof(param_name), "%s_TIM%d", _mixing_output.paramPrefix(), timer);
		timing_param_handles[timer] = param_find(param_name);

		if (timing_param_handles[timer] != PARAM_INVALID) {
			(void)param_get(timing_param_handles[timer], &tim_config[timer]);
		}
	}

	for (int group = 0; group < PWMESC_DSHOT_N_TIMER_GROUPS; ++group) {
		snprintf(param_name, sizeof(param_name), "%s_BIDIR%d", _mixing_output.paramPrefix(), group);
		dshot_bidir_param_handles[group] = param_find(param_name);
	}

	for (int timer = PWMESC_DSHOT_N_TIMER_GROUPS; timer < 4; ++timer) {
		if (tim_config[timer] <= PROTOCOL_DSHOT1200 && tim_config[timer] >= PROTOCOL_DSHOT150) {
			PX4_ERR("TIM%d does not support DShot with pwm_esc, reverting to PWM %d Hz", timer, PWM_DEFAULT_RATE);

			if (timing_param_handles[timer] != PARAM_INVALID) {
				int32_t pwm_mode = PWM_DEFAULT_RATE;
				param_set_no_notification(timing_param_handles[timer], &pwm_mode);
				tim_config[timer] = pwm_mode;
			}
		}
	}

	for (int timer = 0; timer < PWMESC_DSHOT_N_TIMER_GROUPS; ++timer) {
		if (tim_config[timer] <= PROTOCOL_DSHOT1200 && tim_config[timer] >= PROTOCOL_DSHOT150) {
			supported_group_selected[timer] = true;

			if (!dshot_requested) {
				dshot_requested = true;
				dshot_mode = tim_config[timer];

			} else if (tim_config[timer] != dshot_mode) {
				PX4_WARN("TIM%d protocol mismatch, enforcing single DShot speed", timer);

				if (timing_param_handles[timer] != PARAM_INVALID) {
					param_set_no_notification(timing_param_handles[timer], &dshot_mode);
					tim_config[timer] = dshot_mode;
				}
			}
		}
	}

	if (supported_group_selected[1] && !supported_group_selected[0]) {
		PX4_ERR("TIM1 DShot on pwm_esc requires TIM0 DShot (contiguous AUX1..N), reverting TIM1 to PWM %d Hz",
			PWM_DEFAULT_RATE);

		if (timing_param_handles[1] != PARAM_INVALID) {
			int32_t pwm_mode = PWM_DEFAULT_RATE;
			param_set_no_notification(timing_param_handles[1], &pwm_mode);
			tim_config[1] = pwm_mode;
		}

		supported_group_selected[1] = false;
		dshot_requested = supported_group_selected[0];
	}

	for (int group = 0; group < PWMESC_DSHOT_N_TIMER_GROUPS; ++group) {
		if (supported_group_selected[group]) {
			dshot_nchannels += dshot_group_nchannels;
		}
	}

	for (int group = 0; group < PWMESC_DSHOT_N_TIMER_GROUPS; ++group) {
		if (supported_group_selected[group] && dshot_bidir_param_handles[group] != PARAM_INVALID) {
			int32_t bidir = 0;

			if (param_get(dshot_bidir_param_handles[group], &bidir) == PX4_OK && bidir != 0) {
				dshot_bidir = true;
			}
		}
	}

	if (dshot_requested && _dshot_fd >= 0) {
		switch (dshot_mode) {
		case PROTOCOL_DSHOT150:
			_dshot_speed = DSHOT_SPEED_150;
			break;

		case PROTOCOL_DSHOT300:
			_dshot_speed = DSHOT_SPEED_300;
			break;

		case PROTOCOL_DSHOT1200:
			_dshot_speed = DSHOT_SPEED_1200;
			break;

		case PROTOCOL_DSHOT600:
		default:
			_dshot_speed = DSHOT_SPEED_600;
			break;
		}

		struct dshot_config_s config {};

		config.speed = _dshot_speed;

		config.bidir = dshot_bidir ? 1 : 0;

		config.active_mask = 1;//(1u << dshot_nchannels) - 1u;

		if (::ioctl(_dshot_fd, DSHOTIOC_CONFIGURE, (unsigned long)((uintptr_t)&config)) < 0) {
			PX4_ERR("DShot configure failed: %d", errno);
			dshot_requested = false;
		}
	}

	_dshot_enabled = dshot_requested;
	_dshot_nchannels = _dshot_enabled ? dshot_nchannels : 0;
	_dshot_bidir_enabled = _dshot_enabled && dshot_bidir;
	_dshot_output_mask = _dshot_enabled ? ((1u << _dshot_nchannels) - 1u) : 0;
	_pwm_output_mask = all_output_mask() & ~_dshot_output_mask;

	if (!_dshot_enabled) {
		_dshot_armed = false;
	}

#endif
}

int
PWMESC::init_pwm_outputs()
{
	int ret = -1;

	_mixing_output.setIgnoreLockdown(_hitl_mode);
	_mixing_output.setMaxNumOutputs(PWMESC_N_CHANNELS);
	update_protocol_config();
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

			ret = set_defaults_for_device(i, _pwm_fd[i], PWMIOC_START);

			if (ret != 0) {
				PX4_ERR("Init failed for %s errno %d", pwm_device_name, errno);
				break;
			}
		}
	}

	/* Open the DSHOT devnode */

	_dshot_fd = ::open(PWMESC_DSHOT_PATH, O_RDONLY);

	if (ret != 0) {
		PX4_ERR("PWM init failed");
	}

	return 0;
	//	return ret;
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
	update_protocol_config();
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
