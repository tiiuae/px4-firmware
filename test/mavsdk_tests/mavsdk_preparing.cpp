
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <string>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <atomic>
#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

template<typename Rep>
bool poll_condition_with_timeout_(
	std::function<bool()> fun, std::chrono::duration<Rep> duration)
{
	static constexpr unsigned check_resolution = 100;

	const std::chrono::microseconds duration_us(duration);
	// Nothing is connected yet. Use the host time.
	const auto start_time = std::chrono::steady_clock::now();

	while (!fun()) {
		std::this_thread::sleep_for(duration_us / check_resolution);
		const auto elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
						start_time);

		if (elapsed_time_us > duration_us) {
			std::cout << "Timeout, waiting for the vehicle for "
					<< elapsed_time_us.count() * std::chrono::steady_clock::period::num
					/ static_cast<double>(std::chrono::steady_clock::period::den)
					<< " seconds\n";
			return false;
		}
	}
	return true;
}

// --url udp://192.168.0.3:14550
int main(int argc, char **argv)
{
	//std::string connection_url = "udp://192.168.0.3:14550";
	std::string connection_url {};
	std::string command{};
	for (int i = 0; i < argc; ++i) {
		const std::string argv_string(argv[i]);
		if (argv_string == "--url" && (argc > (i + 1))) {
			connection_url = argv[i + 1];
			i++;
		}
		else if (argv_string == "--command" && (argc > (i + 1))) {
			command = argv[i + 1];
			i++;
		}
	}

	if (connection_url.empty()) {
		std::cerr << "No connection URL  was supplied" << std::endl;
		return 1;
	}

   	Mavsdk mavsdk{};
	std::cout << "connection url " << connection_url << std::endl;
	ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);
	if (connection_result != ConnectionResult::Success)
	{
		std::cerr << "Connect was failed" << std::endl;
		return 1;
	}
	std::cout << "Waiting for system connect" << std::endl;

    	if(!poll_condition_with_timeout_(
	[&]() { return mavsdk.systems().size() > 0; }, std::chrono::seconds(25)))
	{
		std::cerr << "Polling was failed" << std::endl;
		return 1;
	}

	auto system = mavsdk.systems().at(0);

	if(!system)
	{
		std::cerr << "The system wasn't be found" << std::endl;
		return 1;
	}

	if ("check" == command)
	{
		std::cout << "Success! The connection was checked." << std::endl;
		return 0;
	}

	auto action = Action{system};
	if ( Action::Result::Success != action.reboot())
	{
		std::cerr << "Reboot doesn't work" << std::endl;
		return 1;
	}
	std::cout << "Rebooting...\n";

	return 0;
}
