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

	// Component names array matching health_component_t enum
	static const char *component_names[] = {
		"Absolute Pressure",       // 1 << 0
		"Accelerometer",           // 1 << 1
		"Airspeed",                // 1 << 2
		"Battery",                 // 1 << 3
		"Camera",                  // 1 << 4
		"Distance Sensor",         // 1 << 5
		"GPS",                     // 1 << 6
		"Gyroscope",               // 1 << 7
		"Magnetometer",            // 1 << 8
		"Motor/ESC",               // 1 << 9
		"Parachute",               // 1 << 10
		"RC Receiver",             // 1 << 11
		"System",                  // 1 << 12
		"Optical Flow",            // 1 << 13
		"Differential Pressure",   // 1 << 14
		"Navigation Computer",     // 1 << 15
		"Open Drone ID",           // 1 << 16
		"Avoidance",               // 1 << 17
		"Payload",                 // 1 << 18
		"Manual Control",          // 1 << 19
		"Home Position",           // 1 << 20
		"Mission",                 // 1 << 21
		"Mode",                    // 1 << 22
		"Arm Authorization",       // 1 << 23
		"SD Card",                 // 1 << 24
		"Wind",                    // 1 << 25
		"Flight Time",             // 1 << 26
		"Geofence",                // 1 << 27
		"VTOL",                    // 1 << 28
		"Offboard",                // 1 << 29
		"Local Position Estimate", // 1 << 30
		"Global Position Estimate" // 1 << 31
	};

	constexpr int max_components = sizeof(component_names) / sizeof(component_names[0]);

	// Print error components
	for (int i = 0; i < max_components && i < 64; i++) {
		if (error_flags & (1ULL << i)) {
			mavlink_log_critical(mavlink_log_pub, "Preflight Check Failed: %s", component_names[i]);
		}
	}

	// Print warning components
	for (int i = 0; i < max_components && i < 64; i++) {
		if (warning_flags & (1ULL << i)) {
			mavlink_log_warning(mavlink_log_pub, "Preflight Check Warning: %s", component_names[i]);
		}
	}
}
