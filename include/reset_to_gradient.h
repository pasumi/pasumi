#pragma once

#include <deque>

#include "resetter.h"

class reset_to_gradient : public resetter {
	public:
		reset_to_gradient();
		~reset_to_gradient();
		float reset(simulation_state& sim_state, user* egocentric_user);

	private:
		vec2f compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos);
		float compute_repulsive_force(vec2f pos, std::vector<object*> objects);

		const int GRADIENT_SAMPLE_RATE = 120; // Number of points around the user to check the value of the potential field
		const int SFR2G_MAX_STEPS = 5; // Number of steps into the future we look for SFR2G reset policy
		const float SFR2G_STEP_SIZE = 0.1f; // Size of each step of the SFR2G reset policy

		std::vector<float> theta_values;
		vec2f steer_target;
		vec2f gradient_dir;
};