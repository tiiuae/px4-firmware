#pragma once
#include <cstdint>
#include <ctime>
inline uint64_t hrt_absolute_time()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}
