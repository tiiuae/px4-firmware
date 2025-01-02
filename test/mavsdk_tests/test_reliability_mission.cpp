#include "autopilot_tester.h"
#include <chrono>
#include <fstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
const int landing_time_s = 45;


void get_test_configuration(const std::string &test_case_name, int &flight_duration_minutes, int &flight_leg_length_m);
int get_one_leg_delay_sec(int flight_leg_length_m);
void save_mission_path(const std::string &test_case_name, Mission::MissionItem &item, bool rtl_at_end);

template<typename T>
bool getElement(json& data, std::string dictionary, std::string name, T& elem);

static const std::string long_mission_reliability_test = "Fly a long mission for checking reliability";
TEST_CASE(long_mission_reliability_test, "[multicopter]")
{
	int flight_duration_minutes = 30;
	int flight_leg_length_m = 100;

	get_test_configuration(long_mission_reliability_test, flight_duration_minutes, flight_leg_length_m);
	const int one_leg_delay_sec = get_one_leg_delay_sec(flight_leg_length_m);

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = false;
	mission_options.fly_through = true;
	mission_options.leg_length_m = flight_leg_length_m;

	tester.arm();
	auto start_time = std::chrono::steady_clock::now();
    	auto duration = std::chrono::minutes(flight_duration_minutes);
	int waypoint = 0;

    	while (std::chrono::steady_clock::now() - start_time < duration)
	{
		tester.prepare_next_random_waypoint_of_mission(mission_options);
		//todo: Save mission waypoints, for repeating
		tester.execute_mission();
		tester.wait_until_hovering(std::chrono::seconds(one_leg_delay_sec));
		std::cout << "<====> Finished waypoint #" << ++waypoint << std::endl;
	}

	tester.execute_rtl();
        std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(one_leg_delay_sec * waypoint + landing_time_s);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

static const std::string round_area_reliability_test = "Fly a long mission for checking reliability in a round area";
TEST_CASE(round_area_reliability_test, "[multicopter]")
{
	int flight_duration_minutes = 10;
	int flight_leg_length_m = 100;

	get_test_configuration(round_area_reliability_test, flight_duration_minutes, flight_leg_length_m);
	const int one_leg_delay_sec = get_one_leg_delay_sec(flight_leg_length_m);

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = false;
	mission_options.fly_through = true;
	mission_options.leg_length_m = flight_leg_length_m;

	tester.arm();

	auto start_time = std::chrono::steady_clock::now();
    	auto duration = std::chrono::minutes(flight_duration_minutes);
	int waypoint = 0;

    	while (std::chrono::steady_clock::now() - start_time < duration)
	{
		auto waypoint_item = tester.prepare_next_random_waypoint_of_round_area_mission(mission_options);
		save_mission_path(round_area_reliability_test, waypoint_item, mission_options.rtl_at_end);
		tester.execute_mission();
		const int two_radios_case_delay = 2;
		tester.wait_until_hovering(std::chrono::seconds(one_leg_delay_sec * two_radios_case_delay));
		std::cout << "<====> Finished waypoint #" << ++waypoint << std::endl;
	}

	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(one_leg_delay_sec + landing_time_s);
	tester.wait_until_disarmed(until_disarmed_timeout);
}
// Add test case with already generated list of waypoints (or used in another failed test case)


int get_one_leg_delay_sec(int flight_leg_length_m)
{
	//TODO: maybe speed should be set
	const float avarage_speed_mc__m_s = 5.0;
	return (flight_leg_length_m / avarage_speed_mc__m_s) * 1.5;
}

template<typename T>
bool getElement(json& data, std::string dictionary, std::string name, T& elem) {
    auto tmp = data.find(dictionary);
    if (tmp != data.end()) {
	auto value = data.at(dictionary).find(name);
	if (value != data.at(dictionary).end()){
        	elem = data.at(dictionary).at(name);
		return true;
	}
    }
    return false;
}

void get_test_configuration(const std::string &test_case_name, int &flight_duration_minutes, int &flight_leg_length_m) {
	const int flight_duration_minuses_max = 60*24*3;
	const int flight_leg_length_m_max = 1000;

	std::ifstream f(config_file_test);
	if (f.is_open())
	{
    		json data = json::parse(f);
		getElement(data, test_case_name, "time", flight_duration_minutes);
		getElement(data, test_case_name, "leg", flight_leg_length_m);
		flight_duration_minutes = flight_duration_minutes > flight_duration_minuses_max ?
		                          flight_duration_minuses_max : flight_duration_minutes;
		flight_leg_length_m = flight_leg_length_m > flight_leg_length_m_max ?
		                      flight_leg_length_m_max : flight_leg_length_m;
		f.close();
	}
	std::cout << "===================================================\n";
	std::cout << "Start mission \n#"<< test_case_name << "\nwith leg " << flight_leg_length_m
	          << " meters and duration "  << flight_duration_minutes << " min\n";
}

void save_mission_path(const std::string &test_case_name, Mission::MissionItem &item, bool rtl_at_end)
{
	const std::string log_file_test {"log_mission.json"};
	json j;
	std::ifstream input_file(log_file_test);
	if (input_file.is_open()) {
		input_file >> j;
		input_file.close();
	}

	const std::string mission_items = {"mission_items"};

	if (!j.contains(test_case_name)) {
		j[test_case_name]["rtl_at_end"] = rtl_at_end;
		j[test_case_name]["is_fly_through"] = item.is_fly_through;
		j[test_case_name]["relative_altitude_m"] = item.relative_altitude_m;
        	j[test_case_name][mission_items] = json::array();
   	}

	j[test_case_name][mission_items].push_back({{"latitude_deg", item.latitude_deg},
	                                            {"longitude_deg",item.longitude_deg }});

	std::ofstream output_file(log_file_test);
	if (!output_file.is_open()) {
		std::cerr << "Could not open the file for writing!" << std::endl;
		return;
	}

	output_file << j.dump(4);
	output_file.close();

	std::cout << "Elements added successfully!" << std::endl;

}
