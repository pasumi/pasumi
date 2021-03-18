#pragma once

#include <vector>

class user;

#include "simulation_state.h"
#include "redirector.h"
#include "user.hpp"
#include "visibility.hpp"
#include "visibility_polygon.h"

class no_rdw : public redirector {
	public:
		no_rdw(resetter* _resetter);
		redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
		std::vector<std::vector<vec2f>> get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue);

	private:
		vec2f compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos);
		float compute_repulsive_force(vec2f pos, std::vector<object*> objects);
		visibility_polygon get_current_visibility_polygon(vec2f pos, environment* env);
		void update_losses(visibility_polygon& phys_poly, visibility_polygon& virt_poly, simulation_state& sim_state, user* egocentric_user);

		const int GRADIENT_SAMPLE_RATE = 120; // Number of points around the user to check the value of the potential field
		const int SFR2G_MAX_STEPS = 5; // Number of steps into the future we look for SFR2G reset policy
		const float SFR2G_STEP_SIZE = 0.1f; // Size of each step of the SFR2G reset policy

		std::vector<float> theta_values;

		vec2f center = vec2f(0.0f, 0.0f);
		vec2f steer_target;
		vec2f gradient_dir;
		float cur_north_loss;
		float cur_east_loss;
		float cur_west_loss;
};