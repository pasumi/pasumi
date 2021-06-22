#pragma once

#include <time.h>

/**
 * Namespace that keeps track of the timesteps that have been simulated. This 
 * namespace basically just keeps track of the number of timesteps/frames that
 * have passed.
 */
namespace timestep {
	const float dt = 0.05f;
	extern float elapsed_time;
	extern int num_timesteps;
	extern time_t start_time;

	/**
	 * Increment the timestep counter by one and the elapsed simulated time by
	 * one timestep.
	 */
	void add_time();
}