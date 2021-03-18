#pragma once

#include <vector>

class user;

#include "simulation_state.h"
#include "redirector.h"
#include "user.hpp"

// Implementation of "Effects of Tracking Area Shape and Size on Artificial Potential Field Redirected Walking

struct segment_container {
	segment_container(vec2f p1, vec2f p2, wall* w) {
		this->p1 = p1;
		this->p2 = p2;
		this->parent_wall = w;
		this->center = (p1 + p2) / 2.0f;
		this->seg_length = length(p1 - p2);
		vec2f temp = p2 - p1;
		// Since the points to all polygons are passed in in CCW order,
		// we know that the outward normal will be the vector derived from
		// rotating the edge 90 deg CW.
		//ccw = (-y, x)
		//cw = (y, -x)
		this->normal = normalize(vec2f(temp.y, -temp.x));
	}

	void flip_normal() {
		normal = normal * -1;
	}

	wall* parent_wall;
	vec2f p1;
	vec2f p2;
	vec2f center;
	vec2f normal; // Points towards the interior of the environment
	float seg_length;
};

class apf_vec : public redirector {
	public:
		apf_vec(physical_environment* phys_env, resetter* _resetter);
		redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* user);
		std::vector<std::vector<vec2f>> get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue);

		vec2f force_vector;
		float steering_rate;

	private:
		redirection_unit set_gains();
		void process_wall(wall* wall, bool flip_norm);
		vec2f compute_force_vector(vec2f phys_pos, vec2f phys_heading, std::vector<user*> other_users);
		float compute_steering_rate(std::deque<proximity_container*> prox_queue);
		redirection_unit set_gains(trajectory_unit cur_move, vec2f phys_heading, vec2f force_vector, float steering_rate);

		const float SEGMENT_LENGTH = 1.0f; // meters
		const float FORCE_DISTANCE_SCALE = 0.00897f;
		const float OBSTACLE_FALLOFF_FACTOR = 2.656f;
		const float USER_FALLOFF_FACTOR = 3.091f;
		const float TURN_ARC_RADIUS = 7.5F; // meters
		const float MAX_STEERING_RATE = 15.0f; // deg/sec
		const float MIN_STEERING_RATE = 1.5f; // deg/sec
		const float BASELINE_ROTA_RATE = 1.5f; // deg/sec

		std::vector<segment_container> static_segments;
		std::vector<segment_container> dynamic_segments;
};