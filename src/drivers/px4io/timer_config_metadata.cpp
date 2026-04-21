/*
 * Metadata-only timer configuration for px4io actuator output generation.
 *
 * This file is parsed by Tools/module_config/output_groups_from_timer_config.py
 * during parameter and actuator metadata generation.
 *
 * px4io has 8 PWM outputs grouped as 4 timer groups with 2 channels each
 * (MAIN 1-2, 3-4, 5-6, 7-8), independent of board FMU timer topology.
 */

io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer1),
	initIOTimer(Timer::Timer2),
	initIOTimer(Timer::Timer3),
	initIOTimer(Timer::Timer4),
};

timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortA, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel1}, {GPIO::PortA, GPIO::Pin2}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel2}, {GPIO::PortA, GPIO::Pin3}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel1}, {GPIO::PortA, GPIO::Pin4}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel2}, {GPIO::PortA, GPIO::Pin5}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortA, GPIO::Pin6}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortA, GPIO::Pin7}),
};
