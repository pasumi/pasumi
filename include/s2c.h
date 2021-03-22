#pragma once

class user;

#include "redirector.h"
#include "simulation_state.h"
#include "user.hpp"

class s2c : public redirector {
	public:
		s2c(resetter* _resetter);
		redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
		std::vector<std::vector<vec2f>> get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue);

	private:
		redirection_unit set_gains(float dx, float dy, float dtheta, float phys_heading, vec2f phys_pos);
		float attenuate_rotation_gain(float angle_to_center, float distance_to_center, float rota_gain);
		float attenuate_curvature_gain(float angle_to_center, float distance_to_center, float curve_gain);

		visibility_polygon get_current_visibility_polygon(vec2f pos, environment* env);
		void update_losses(visibility_polygon& phys_poly, visibility_polygon& virt_poly, simulation_state& sim_state, user* egocentric_user);

		const float BASELINE_ROTATION = 0.1f; // deg/sec
		const float TRANSLATIONAL_ROTATION = 7.5f; // deg/sec for user walking at 1m/s
		const float SMOOTHING_FACTOR = 0.125f;
		const float DISTANCE_DAMPING_THRESHOLD = 1.25f;

		vec2f target;
		vec2f center = vec2f(0.0f, 0.0f);
		vec2f temp_target;
		float cur_north_loss;
		float cur_east_loss;
		float cur_west_loss;
		float prev_rota_gain;
};