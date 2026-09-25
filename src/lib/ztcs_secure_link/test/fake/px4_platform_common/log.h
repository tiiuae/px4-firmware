#pragma once
#include <cstdio>
#define PX4_ERR(...)  do { fprintf(stderr, "ERR  "); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)
#define PX4_INFO(...) do { fprintf(stderr, "INFO "); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)
