/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include "GZBridge.hpp"

#include <uORB/Subscription.hpp>

#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/getopt.h>

#include <lib/drivers/device/Device.hpp>

#include <iostream>
#include <string>

GZBridge::GZBridge(const char *world, const char *name, const char *model,
		   const char *type, const char *pose_str) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_world_name(world),
	_model_name(name),
	_model_sim(model),
	_vehicle_type(type),
	_model_pose(pose_str)
{
	pthread_mutex_init(&_mutex, nullptr);

	updateParams();
}

GZBridge::~GZBridge()
{
	// TODO: unsubscribe

	for (auto &sub_topic : _node.SubscribedTopics()) {
		_node.Unsubscribe(sub_topic);
	}
}

int GZBridge::init()
{
	if (!_model_sim.empty()) {

		// service call to create model
		// ign service -s /world/${PX4_GZ_WORLD}/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req "sdf_filename: \"${PX4_GZ_MODEL}/model.sdf\""
		gz::msgs::EntityFactory req{};
		req.set_sdf_filename(_model_sim + "/model.sdf");

		req.set_name(_model_name); // New name for the entity, overrides the name on the SDF.

		req.set_allow_renaming(false); // allowed to rename the entity in case of overlap with existing entities

		if (!_model_pose.empty()) {
			PX4_INFO("Requested Model Position: %s", _model_pose.c_str());

			std::vector<double> model_pose_v;

			std::stringstream ss(_model_pose);

			while (ss.good()) {
				std::string substr;
				std::getline(ss, substr, ',');
				model_pose_v.push_back(std::stod(substr));
			}

			while (model_pose_v.size() < 6) {
				model_pose_v.push_back(0.0);
			}

			gz::msgs::Pose *p = req.mutable_pose();
			gz::msgs::Vector3d *position = p->mutable_position();
			position->set_x(model_pose_v[0]);
			position->set_y(model_pose_v[1]);
			position->set_z(model_pose_v[2]);

			gz::math::Quaterniond q(model_pose_v[3], model_pose_v[4], model_pose_v[5]);

			q.Normalize();
			gz::msgs::Quaternion *orientation = p->mutable_orientation();
			orientation->set_x(q.X());
			orientation->set_y(q.Y());
			orientation->set_z(q.Z());
			orientation->set_w(q.W());
		}

		//world/$WORLD/create service.
		gz::msgs::Boolean rep;
		bool result;
		std::string create_service = "/world/" + _world_name + "/create";

		if (_node.Request(create_service, req, 1000, rep, result)) {
			if (!rep.data() || !result) {
				PX4_ERR("EntityFactory service call failed");
				return PX4_ERROR;
			}

		} else {
			PX4_ERR("Service call timed out");
			return PX4_ERROR;
		}
	}

	// clock
	std::string clock_topic = "/world/" + _world_name + "/clock";

	if (!_node.Subscribe(clock_topic, &GZBridge::clockCallback, this)) {
		PX4_ERR("failed to subscribe to %s", clock_topic.c_str());
		return PX4_ERROR;
	}

	// pose: /world/$WORLD/pose/info
	std::string world_pose_topic = "/world/" + _world_name + "/pose/info";

	if (!_node.Subscribe(world_pose_topic, &GZBridge::poseInfoCallback, this)) {
		PX4_ERR("failed to subscribe to %s", world_pose_topic.c_str());
		return PX4_ERROR;
	}

	// IMU: /world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu
	std::string imu_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/imu_sensor/imu";

	if (!_node.Subscribe(imu_topic, &GZBridge::imuCallback, this)) {
		PX4_ERR("failed to subscribe to %s", imu_topic.c_str());
		return PX4_ERROR;
	}

	// MAG: /world/$WORLD/model/$MODEL/link/base_link/sensor/magnetometer_sensor/magnetometer
	std::string mag_topic = "/world/" + _world_name + "/model/" + _model_name +
				"/link/base_link/sensor/magnetometer_sensor/magnetometer";

	if (!_node.Subscribe(mag_topic, &GZBridge::magnetometerCallback, this)) {
		PX4_ERR("failed to subscribe to %s", mag_topic.c_str());
		return PX4_ERROR;
	}

	// GPS: /world/$WORLD/model/$MODEL/link/gps1_link/sensor/gps/navsat
	std::string gps_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/gps1_link/sensor/gps/navsat";

	if (!_node.Subscribe(gps_topic, &GZBridge::gpsCallback, this)) {
		PX4_ERR("failed to subscribe to %s", gps_topic.c_str());
		return PX4_ERROR;
	}

	// Air pressure: /world/$WORLD/model/$MODEL/link/base_link/sensor/air_pressure_sensor/air_pressure
	std::string air_pressure_topic = "/world/" + _world_name + "/model/" + _model_name +
					 "/link/base_link/sensor/air_pressure_sensor/air_pressure";

	if (!_node.Subscribe(air_pressure_topic, &GZBridge::barometerCallback, this)) {
		PX4_ERR("failed to subscribe to %s", air_pressure_topic.c_str());
		return PX4_ERROR;
	}

#if 0
	// Airspeed: /world/$WORLD/model/$MODEL/link/airspeed_link/sensor/air_speed/air_speed
	std::string airpressure_topic = "/world/" + _world_name + "/model/" + _model_name +
					"/link/airspeed_link/sensor/air_speed/air_speed";

	if (!_node.Subscribe(airpressure_topic, &GZBridge::airspeedCallback, this)) {
		PX4_ERR("failed to subscribe to %s", airpressure_topic.c_str());
		return PX4_ERROR;
	}

#endif

	// ESC feedback: /x500/command/motor_speed
	if (_vehicle_type != "rover") {
		std::string motor_speed_topic = "/" + _model_name + "/command/motor_speed";

		if (!_node.Subscribe(motor_speed_topic, &GZBridge::motorSpeedCallback, this)) {
			PX4_ERR("failed to subscribe to %s", motor_speed_topic.c_str());
			return PX4_ERROR;
		}
	}

	// list all subscriptions
	for (auto &sub_topic : _node.SubscribedTopics()) {
		PX4_INFO("subscribed: %s", sub_topic.c_str());
	}

	// output (rover vehicle type) eg /model/$MODEL_NAME/cmd_vel
	if (_vehicle_type == "rover") {
		std::string cmd_vel_topic = "/model/" + _model_name + "/cmd_vel";
		_cmd_vel_pub = _node.Advertise<gz::msgs::Twist>(cmd_vel_topic);

		if (!_cmd_vel_pub.Valid()) {
			PX4_ERR("failed to advertise %s", cmd_vel_topic.c_str());
			return PX4_ERROR;
		}

	} else {
		// output (other vehicle types) eg /X500/command/motor_speed
		std::string actuator_topic = "/" + _model_name + "/command/motor_speed";
		_actuators_pub = _node.Advertise<gz::msgs::Actuators>(actuator_topic);

		if (!_actuators_pub.Valid()) {
			PX4_ERR("failed to advertise %s", actuator_topic.c_str());
			return PX4_ERROR;
		}
	}

	ScheduleNow();
	return OK;
}

int GZBridge::task_spawn(int argc, char *argv[])
{
	const char *world_name = "default";
	const char *model_name = nullptr;
	const char *model_pose = nullptr;
	const char *model_sim = nullptr;
	const char *vehicle_type = nullptr;
	const char *px4_instance = nullptr;


	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "w:m:p:i:n:v:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'w':
			// world
			world_name = myoptarg;
			break;

		case 'n':
			// model name
			model_name = myoptarg;
			break;

		case 'p':
			// pose
			model_pose = myoptarg;
			break;

		case 'm':
			// model type
			model_sim = myoptarg;
			break;

		case 'i':
			// pose
			px4_instance = myoptarg;
			break;

		case 'v':
			// vehicle type
			vehicle_type = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!model_pose) {
		model_pose = "";
	}

	if (!model_sim) {
		model_sim = "";
	}

	if (!vehicle_type) {
		vehicle_type = "mc";
	}

	if (!px4_instance) {
		if (!model_name) {
			model_name = model_sim;
		}

	} else if (!model_name) {
		std::string model_name_std = std::string(model_sim) + "_" + std::string(px4_instance);
		model_name = model_name_std.c_str();
	}

	PX4_INFO("world: %s, model name: %s, simulation model: %s, vehicle type: %s", world_name, model_name, model_sim,
		 vehicle_type);

	GZBridge *instance = new GZBridge(world_name, model_name, model_sim, vehicle_type, model_pose);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			// lockstep scheduler wait for initial clock set before returning
			int sleep_count_limit = 1000;

			while ((instance->world_time_us() == 0) && sleep_count_limit > 0) {
				// wait for first clock message
				system_usleep(1000);
				sleep_count_limit--;
			}

			if (instance->world_time_us() == 0) {
				PX4_ERR("timed out waiting for clock message");
				instance->request_stop();
				instance->ScheduleNow();

			} else {
				return PX4_OK;
			}

#else
			return PX4_OK;
#endif // ENABLE_LOCKSTEP_SCHEDULER

			//return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool GZBridge::updateClock(const uint64_t tv_sec, const uint64_t tv_nsec)
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	struct timespec ts;
	ts.tv_sec = tv_sec;
	ts.tv_nsec = tv_nsec;

	if (px4_clock_settime(CLOCK_MONOTONIC, &ts) == 0) {
		_world_time_us.store(ts_to_abstime(&ts));
		return true;
	}

#endif // ENABLE_LOCKSTEP_SCHEDULER

	return false;
}

void GZBridge::clockCallback(const gz::msgs::Clock &clock)
{
	pthread_mutex_lock(&_mutex);

	const uint64_t time_us = (clock.sim().sec() * 1000000) + (clock.sim().nsec() / 1000);

	if (time_us > _world_time_us.load()) {
		updateClock(clock.sim().sec(), clock.sim().nsec());
	}

	pthread_mutex_unlock(&_mutex);
}

void GZBridge::imuCallback(const gz::msgs::IMU &imu)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);

	const uint64_t time_us = (imu.header().stamp().sec() * 1000000) + (imu.header().stamp().nsec() / 1000);

	if (time_us > _world_time_us.load()) {
		updateClock(imu.header().stamp().sec(), imu.header().stamp().nsec());
	}

	// FLU -> FRD
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					     imu.linear_acceleration().x(),
					     imu.linear_acceleration().y(),
					     imu.linear_acceleration().z()));

	// publish accel
	sensor_accel_s sensor_accel{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	sensor_accel.timestamp_sample = time_us;
	sensor_accel.timestamp = time_us;
#else
	sensor_accel.timestamp_sample = hrt_absolute_time();
	sensor_accel.timestamp = hrt_absolute_time();
#endif
	sensor_accel.device_id = 1310988; // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	sensor_accel.x = accel_b.X();
	sensor_accel.y = accel_b.Y();
	sensor_accel.z = accel_b.Z();
	sensor_accel.temperature = NAN;
	sensor_accel.samples = 1;
	_sensor_accel_pub.publish(sensor_accel);


	gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					    imu.angular_velocity().x(),
					    imu.angular_velocity().y(),
					    imu.angular_velocity().z()));

	// publish gyro
	sensor_gyro_s sensor_gyro{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	sensor_gyro.timestamp_sample = time_us;
	sensor_gyro.timestamp = time_us;
#else
	sensor_gyro.timestamp_sample = hrt_absolute_time();
	sensor_gyro.timestamp = hrt_absolute_time();
#endif
	sensor_gyro.device_id = 1310988; // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	sensor_gyro.x = gyro_b.X();
	sensor_gyro.y = gyro_b.Y();
	sensor_gyro.z = gyro_b.Z();
	sensor_gyro.temperature = NAN;
	sensor_gyro.samples = 1;
	_sensor_gyro_pub.publish(sensor_gyro);

	pthread_mutex_unlock(&_mutex);
}

void GZBridge::magnetometerCallback(const gz::msgs::Magnetometer &magnetometer)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);

	const uint64_t time_us = (magnetometer.header().stamp().sec() * 1000000) + (magnetometer.header().stamp().nsec() /
				 1000);

	if (time_us > _world_time_us.load()) {
		updateClock(magnetometer.header().stamp().sec(), magnetometer.header().stamp().nsec());
	}

	// FLU -> FRD
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	gz::math::Vector3d mag_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					   magnetometer.field_tesla().x(),
					   magnetometer.field_tesla().y(),
					   magnetometer.field_tesla().z()));

	// publish mag
	sensor_mag_s sensor_mag{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	sensor_mag.timestamp_sample = time_us;
	sensor_mag.timestamp = time_us;
#else
	sensor_mag.timestamp_sample = hrt_absolute_time();
	sensor_mag.timestamp = hrt_absolute_time();
#endif
	sensor_mag.device_id = 197388; // 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	sensor_mag.x = mag_b.X();
	sensor_mag.y = mag_b.Y();
	sensor_mag.z = mag_b.Z();
	sensor_mag.temperature = NAN;
	sensor_mag.error_count = 0;

	_sensor_mag_pub.publish(sensor_mag);

	pthread_mutex_unlock(&_mutex);
}

void GZBridge::gpsCallback(const gz::msgs::NavSat &gps)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);

	const uint64_t time_us = (gps.header().stamp().sec() * 1000000) + (gps.header().stamp().nsec() / 1000);

	if (time_us > _world_time_us.load()) {
		updateClock(gps.header().stamp().sec(), gps.header().stamp().nsec());
	}

	// device id
	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	device_id.devid_s.bus = 0;
	device_id.devid_s.address = 0;
	device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

	// publish gps
	sensor_gps_s sensor_gps{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	sensor_gps.timestamp_sample = time_us;
	sensor_gps.timestamp = time_us;
#else
	sensor_gps.timestamp_sample = hrt_absolute_time();
	sensor_gps.timestamp = hrt_absolute_time();
#endif

	sensor_gps.device_id = device_id.devid;

	sensor_gps.lat = roundf(gps.latitude_deg() * 1e7); // Latitude in 1E-7 degrees
	sensor_gps.lon = roundf(gps.longitude_deg() * 1e7); // Longitude in 1E-7 degrees
	sensor_gps.alt = roundf(static_cast<float>(gps.altitude()) *
				1000.f); // Altitude in 1E-3 meters above MSL, (millimetres)
	sensor_gps.alt_ellipsoid = sensor_gps.alt;

	sensor_gps.s_variance_m_s = 0.5f;
	sensor_gps.c_variance_rad = 0.1f;

	sensor_gps.fix_type = 3; // 3D fix

	sensor_gps.eph = 0.9f;
	sensor_gps.epv = 1.78f;
	sensor_gps.hdop = 0.7f;
	sensor_gps.vdop = 1.1f;

	sensor_gps.noise_per_ms = 0;
	sensor_gps.automatic_gain_control = 0;
	sensor_gps.jamming_state = 0;
	sensor_gps.jamming_indicator = 0;

	sensor_gps.vel_m_s = sqrtf(gps.velocity_east() * gps.velocity_east() +
				   gps.velocity_north() * gps.velocity_north());
	sensor_gps.vel_n_m_s = gps.velocity_north();
	sensor_gps.vel_e_m_s = gps.velocity_east();
	sensor_gps.vel_d_m_s = gps.velocity_up();
	sensor_gps.cog_rad = atan2(gps.velocity_east(), gps.velocity_north());
	sensor_gps.vel_ned_valid = true;
	sensor_gps.timestamp_time_relative = 0;
	sensor_gps.time_utc_usec = 0;

	sensor_gps.satellites_used = 8;

	sensor_gps.heading = 0.0f;
	sensor_gps.heading_offset = NAN;
	sensor_gps.heading_accuracy = 0;

	_sensor_gps_pub.publish(sensor_gps);

	pthread_mutex_unlock(&_mutex);
}

void GZBridge::barometerCallback(const gz::msgs::FluidPressure &air_pressure)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);

	const uint64_t time_us = (air_pressure.header().stamp().sec() * 1000000)
				 + (air_pressure.header().stamp().nsec() / 1000);

	// publish
	sensor_baro_s sensor_baro{};
	sensor_baro.timestamp_sample = time_us;
	sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
	sensor_baro.pressure = air_pressure.pressure();
	sensor_baro.temperature = this->_temperature;
	sensor_baro.timestamp = hrt_absolute_time();
	_sensor_baro_pub.publish(sensor_baro);

	pthread_mutex_unlock(&_mutex);
}

#if 0
void GZBridge::airspeedCallback(const gz::msgs::AirSpeedSensor &air_speed)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);

	const uint64_t time_us = (air_speed.header().stamp().sec() * 1000000)
				 + (air_speed.header().stamp().nsec() / 1000);

	double air_speed_value = air_speed.diff_pressure();

	differential_pressure_s report{};
	report.timestamp_sample = time_us;
	report.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
	report.differential_pressure_pa = static_cast<float>(air_speed_value); // hPa to Pa;
	report.temperature = static_cast<float>(air_speed.temperature()) + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // K to C
	report.timestamp = hrt_absolute_time();;
	_differential_pressure_pub.publish(report);

	this->_temperature = report.temperature;

	pthread_mutex_unlock(&_mutex);
}
#endif

void GZBridge::poseInfoCallback(const gz::msgs::Pose_V &pose)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);

	for (int p = 0; p < pose.pose_size(); p++) {
		if (pose.pose(p).name() == _model_name) {

			const uint64_t time_us = (pose.header().stamp().sec() * 1000000) + (pose.header().stamp().nsec() / 1000);

			if (time_us > _world_time_us.load()) {
				updateClock(pose.header().stamp().sec(), pose.header().stamp().nsec());
			}

			const double dt = math::constrain((time_us - _timestamp_prev) * 1e-6, 0.001, 0.1);
			_timestamp_prev = time_us;

			gz::msgs::Vector3d pose_position = pose.pose(p).position();
			gz::msgs::Quaternion pose_orientation = pose.pose(p).orientation();

			static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

			/**
			 * @brief Quaternion for rotation between ENU and NED frames
			 *
			 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
			 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
			 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
			 */
			static const auto q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

			// ground truth
			gz::math::Quaterniond q_gr = gz::math::Quaterniond(
							     pose_orientation.w(),
							     pose_orientation.x(),
							     pose_orientation.y(),
							     pose_orientation.z());

			gz::math::Quaterniond q_gb = q_gr * q_FLU_to_FRD.Inverse();
			gz::math::Quaterniond q_nb = q_ENU_to_NED * q_gb;

			// publish attitude groundtruth
			vehicle_attitude_s vehicle_attitude_groundtruth{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			vehicle_attitude_groundtruth.timestamp_sample = time_us;
#else
			vehicle_attitude_groundtruth.timestamp_sample = hrt_absolute_time();
#endif
			vehicle_attitude_groundtruth.q[0] = q_nb.W();
			vehicle_attitude_groundtruth.q[1] = q_nb.X();
			vehicle_attitude_groundtruth.q[2] = q_nb.Y();
			vehicle_attitude_groundtruth.q[3] = q_nb.Z();
			vehicle_attitude_groundtruth.timestamp = hrt_absolute_time();
			_attitude_ground_truth_pub.publish(vehicle_attitude_groundtruth);

			// publish angular velocity groundtruth
			const matrix::Eulerf euler{matrix::Quatf(vehicle_attitude_groundtruth.q)};
			vehicle_angular_velocity_s vehicle_angular_velocity_groundtruth{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			vehicle_angular_velocity_groundtruth.timestamp_sample = time_us;
#else
			vehicle_angular_velocity_groundtruth.timestamp_sample = hrt_absolute_time();
#endif
			const matrix::Vector3f angular_velocity = (euler - _euler_prev) / dt;
			_euler_prev = euler;
			angular_velocity.copyTo(vehicle_angular_velocity_groundtruth.xyz);

			vehicle_angular_velocity_groundtruth.timestamp = hrt_absolute_time();
			_angular_velocity_ground_truth_pub.publish(vehicle_angular_velocity_groundtruth);

			if (!_pos_ref.isInitialized()) {
				_pos_ref.initReference((double)_param_sim_home_lat.get(), (double)_param_sim_home_lon.get(), hrt_absolute_time());
			}

			vehicle_local_position_s local_position_groundtruth{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
			local_position_groundtruth.timestamp_sample = time_us;
#else
			local_position_groundtruth.timestamp_sample = hrt_absolute_time();
#endif
			// position ENU -> NED
			const matrix::Vector3d position{pose_position.y(), pose_position.x(), -pose_position.z()};
			const matrix::Vector3d velocity{(position - _position_prev) / dt};
			const matrix::Vector3d acceleration{(velocity - _velocity_prev) / dt};

			_position_prev = position;
			_velocity_prev = velocity;

			local_position_groundtruth.ax = acceleration(0);
			local_position_groundtruth.ay = acceleration(1);
			local_position_groundtruth.az = acceleration(2);
			local_position_groundtruth.vx = velocity(0);
			local_position_groundtruth.vy = velocity(1);
			local_position_groundtruth.vz = velocity(2);
			local_position_groundtruth.x = position(0);
			local_position_groundtruth.y = position(1);
			local_position_groundtruth.z = position(2);

			local_position_groundtruth.ref_lat = _pos_ref.getProjectionReferenceLat(); // Reference point latitude in degrees
			local_position_groundtruth.ref_lon = _pos_ref.getProjectionReferenceLon(); // Reference point longitude in degrees
			local_position_groundtruth.ref_alt = _param_sim_home_alt.get();
			local_position_groundtruth.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();

			local_position_groundtruth.timestamp = hrt_absolute_time();
			_lpos_ground_truth_pub.publish(local_position_groundtruth);

			if (_pos_ref.isInitialized()) {
				// publish position groundtruth
				vehicle_global_position_s global_position_groundtruth{};
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
				global_position_groundtruth.timestamp_sample = time_us;
#else
				global_position_groundtruth.timestamp_sample = hrt_absolute_time();
#endif

				_pos_ref.reproject(local_position_groundtruth.x, local_position_groundtruth.y,
						   global_position_groundtruth.lat, global_position_groundtruth.lon);

				global_position_groundtruth.alt = _param_sim_home_alt.get() - static_cast<float>(position(2));
				global_position_groundtruth.timestamp = hrt_absolute_time();
				_gpos_ground_truth_pub.publish(global_position_groundtruth);
			}

			pthread_mutex_unlock(&_mutex);
			return;
		}
	}

	pthread_mutex_unlock(&_mutex);
}

void GZBridge::motorSpeedCallback(const gz::msgs::Actuators &actuators)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);

	esc_status_s esc_status{};
	esc_status.esc_count = actuators.velocity_size();

	for (int i = 0; i < actuators.velocity_size(); i++) {
		esc_status.esc[i].timestamp = hrt_absolute_time();
		esc_status.esc[i].esc_rpm = actuators.velocity(i);
		esc_status.esc_online_flags |= 1 << i;

		if (actuators.velocity(i) > 0) {
			esc_status.esc_armed_flags |= 1 << i;
		}
	}

	if (esc_status.esc_count > 0) {
		esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(esc_status);
	}

	pthread_mutex_unlock(&_mutex);
}

bool GZBridge::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			     unsigned num_control_groups_updated)
{
	unsigned active_output_count = 0;

	for (unsigned i = 0; i < num_outputs; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			active_output_count++;

		} else {
			break;
		}
	}

	if (active_output_count > 0) {
		gz::msgs::Actuators rotor_velocity_message;
		rotor_velocity_message.mutable_velocity()->Resize(active_output_count, 0);

		for (unsigned i = 0; i < active_output_count; i++) {
			rotor_velocity_message.set_velocity(i, outputs[i]);
		}

		if (_actuators_pub.Valid()) {
			return _actuators_pub.Publish(rotor_velocity_message);
		}
	}

	return false;
}

void GZBridge::updateCmdVel()
{
	if (_actuator_controls_sub.updated()) {
		actuator_controls_s actuator_controls_msg;

		if (_actuator_controls_sub.copy(&actuator_controls_msg)) {
			auto throttle = actuator_controls_msg.control[actuator_controls_s::INDEX_THROTTLE];
			auto steering = actuator_controls_msg.control[actuator_controls_s::INDEX_YAW];

			// publish cmd_vel
			gz::msgs::Twist cmd_vel_message;
			cmd_vel_message.mutable_linear()->set_x(throttle);
			cmd_vel_message.mutable_angular()->set_z(steering);

			if (_cmd_vel_pub.Valid()) {
				_cmd_vel_pub.Publish(cmd_vel_message);
			}
		}
	}
}

void GZBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	pthread_mutex_lock(&_mutex);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}

	// In case of rover vehicle type, publish gz cmd_vel
	if (_vehicle_type == "rover") { updateCmdVel(); }

	_mixing_output.update();

	//ScheduleDelayed(1000_us);

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	pthread_mutex_unlock(&_mutex);
}

int GZBridge::print_status()
{
	//perf_print_counter(_cycle_perf);
	if (_vehicle_type != "rover") { _mixing_output.printStatus(); }

	return 0;
}

int GZBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GZBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gz_bridge", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('m', nullptr, nullptr, "Fuel model name", false);
	PRINT_MODULE_USAGE_PARAM_STRING('v', nullptr, nullptr, "Vehicle type (mc, rover, uuv, etc)", false);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, nullptr, "Model Pose", false);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, nullptr, "Model name", false);
	PRINT_MODULE_USAGE_PARAM_STRING('i', nullptr, nullptr, "PX4 instance", false);
	PRINT_MODULE_USAGE_PARAM_STRING('w', nullptr, nullptr, "World name", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gz_bridge_main(int argc, char *argv[])
{
	return GZBridge::main(argc, argv);
}
