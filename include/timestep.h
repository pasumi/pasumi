#pragma once

#include <time.h>

namespace timestep {
	const float dt = 0.05f;
	extern float elapsed_time;
	extern int num_timesteps;
	extern time_t start_time;

	void add_time();
}