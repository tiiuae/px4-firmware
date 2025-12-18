/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include "HealthAndArmingChecks.hpp"

HealthAndArmingChecks::HealthAndArmingChecks(ModuleParams *parent, vehicle_status_s &status)
	: ModuleParams(parent),
	  _context(status)
{
	// Initialize mode requirements to invalid
	_failsafe_flags.angular_velocity_invalid = true;
	_failsafe_flags.attitude_invalid = true;
	_failsafe_flags.local_altitude_invalid = true;
	_failsafe_flags.local_position_invalid = true;
	_failsafe_flags.local_position_invalid_relaxed = true;
	_failsafe_flags.local_velocity_invalid = true;
	_failsafe_flags.global_position_invalid = true;
	_failsafe_flags.auto_mission_missing = true;
	_failsafe_flags.offboard_control_signal_lost = true;
	_failsafe_flags.home_position_invalid = true;
}

bool HealthAndArmingChecks::update(bool force_reporting)
{
	_reporter.reset();

	_reporter.prepare(_context.status().vehicle_type);

	for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
		if (!_checks[i]) {
			break;
		}

		_checks[i]->checkAndReport(_context, _reporter);
	}

	const bool results_changed = _reporter.finalize();
	const bool reported = _reporter.report(_context.isArmed(), force_reporting);

	if (reported) {

		// LEGACY start
		// Run the checks again, this time with the mavlink publication set.
		// We don't expect any change, and rate limitation would prevent the events from being reported again,
		// so we only report mavlink_log_*.
		_reporter._mavlink_log_pub = &_mavlink_log_pub;
		_reporter.reset();

		_reporter.prepare(_context.status().vehicle_type);

		for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
			if (!_checks[i]) {
				break;
			}

			_checks[i]->checkAndReport(_context, _reporter);
		}

		_reporter.finalize();
		_reporter.report(_context.isArmed(), false);
		_reporter._mavlink_log_pub = nullptr;
		// LEGACY end

		health_report_s health_report;
		_reporter.getHealthReport(health_report);
		health_report.timestamp = hrt_absolute_time();
		_health_report_pub.publish(health_report);
	}

	// Check if we need to publish the failsafe flags
	const hrt_abstime now = hrt_absolute_time();

	if ((now > _failsafe_flags.timestamp + 500_ms) || results_changed) {
		_failsafe_flags.timestamp = hrt_absolute_time();
		_failsafe_flags_pub.publish(_failsafe_flags);
	}

	return reported;
}

void HealthAndArmingChecks::updateParams()
{
	for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
		if (!_checks[i]) {
			break;
		}

		_checks[i]->updateParams();
	}
}

void HealthAndArmingChecks::printFailingArmingChecks(orb_advert_t *mavlink_log_pub) const
{
	const auto &arming_checks = _reporter.armingCheckResults();
	const uint64_t error_flags = (uint64_t)arming_checks.error;
	const uint64_t warning_flags = (uint64_t)arming_checks.warning;

	if (error_flags == 0 && warning_flags == 0) {
		return;
	}

	// Component name mapping for each bit position
	// Based on health_component_t enum values which are powers of 2
	static const struct {
		uint32_t value;
		const char *name;
	} component_map[] = {
		{0x1, "None"},                      // Bit 0
		{0x2, "Absolute Pressure"},         // Bit 1
		{0x4, "Differential Pressure"},     // Bit 2
		{0x8, "GPS"},                       // Bit 3
		{0x10, "Optical Flow"},             // Bit 4
		{0x20, "Vision Position"},          // Bit 5
		{0x40, "Distance Sensor"},          // Bit 6
		{0x80, "Remote Control"},           // Bit 7
		{0x100, "Motors/ESCs"},             // Bit 8
		{0x200, "UTM"},                     // Bit 9
		{0x400, "Logging"},                 // Bit 10
		{0x800, "Battery"},                 // Bit 11
		{0x1000, "Communication Links"},    // Bit 12
		{0x2000, "Rate Controller"},        // Bit 13
		{0x4000, "Attitude Controller"},    // Bit 14
		{0x8000, "Position Controller"},    // Bit 15
		{0x10000, "Attitude Estimate"},     // Bit 16
		{0x20000, "Local Position Estimate"}, // Bit 17
		{0x40000, "Mission"},               // Bit 18
		{0x80000, "Avoidance"},             // Bit 19
		{0x100000, "System"},               // Bit 20
		{0x200000, "Camera"},               // Bit 21
		{0x400000, "Gimbal"},               // Bit 22
		{0x800000, "Payload"},              // Bit 23
		{0x1000000, "Global Position Estimate"}, // Bit 24
		{0x2000000, "Storage"},             // Bit 25
		{0x4000000, "Parachute"},           // Bit 26
		{0x8000000, "Magnetometer"},        // Bit 27
		{0x10000000, "Accelerometer"},      // Bit 28
		{0x20000000, "Gyroscope"},          // Bit 29
		{0x40000000, "Open Drone ID"},      // Bit 30
	};

	constexpr int num_components = sizeof(component_map) / sizeof(component_map[0]);

	// Print error components
	for (int i = 0; i < num_components; i++) {
		if (error_flags & component_map[i].value) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Check Failed: %s", component_map[i].name);
		}
	}

	// Print warning components
	for (int i = 0; i < num_components; i++) {
		if (warning_flags & component_map[i].value) {
			mavlink_log_warning(mavlink_log_pub, "Preflight Check Warning: %s", component_map[i].name);
		}
	}
}
