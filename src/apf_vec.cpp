#include <math.h>
#include <iostream>

#include "apf_vec.h"
#include "config.h"

apf_vec::apf_vec(physical_environment* phys_env, resetter* _resetter) {
	name = "APF Force Vector based";
	cur_rota_gain = 0.5f;
	min_rota_gain = 0.85f;
	max_rota_gain = 1.2f;

	// Original paper does not implement translation gains
	min_trans_gain = 0.86f;

	curve_radius = TURN_ARC_RADIUS; // meters
	cur_curve_per_deg = curve_radius_to_deg_per_meter();
	curve_dir = 1; // curve to the left of the user == 1. curve to the right of the user == -1

	for (wall* w : phys_env->get_walls()) {
		process_wall(w, true);
	}
	for (obstacle* o : phys_env->get_obstacles()) {
		for (wall* w : o->get_walls()) {
			process_wall(w, false);
		}
	}
	//assert(dynamic_segments.size() == 0); // The original paper does not support dynamic objects.
	cur_alignment = std::vector<float>{ 0.0f, 0.0f, 0.0f };

	reset_policy = _resetter;
}

void apf_vec::process_wall(wall* wall, bool flip_norm) {
	std::vector<vec2f*> wall_verts = wall->get_vertices();
	vec2f segment_unit_vec = normalize(*wall_verts[1] - *wall_verts[0]) * SEGMENT_LENGTH;
	float wall_length = length(*wall_verts[0] - *wall_verts[1]);
	vec2f* p1;
	vec2f* p2;
	p1 = wall_verts[0];

	for (int i = 0; i < int(wall_length / SEGMENT_LENGTH); i++) {
		p2 = new vec2f(*p1 + segment_unit_vec);
		segment_container s = segment_container(*p1, *p2, wall);
		if (flip_norm) s.flip_normal(); // To force walls of the phys env to point inwards.
		if (wall->mvmt == object::MOVEMENT_TYPE::STATIC) {
			static_segments.push_back(s);
		}
		else {
			dynamic_segments.push_back(s);
		}
		p1 = p2;
	}

	// Wall length is not a multiple of SEGMENT_LENGTH. We need to make
	// another segment with a length smaller than SEGMENT_LENGTH for the 
	// leftover part of the wall not processed in the loop above.
	if (wall_length > int(wall_length)) {
		p2 = wall_verts[1];
		segment_container s = segment_container(*p1, *p2, wall);
		if (wall->mvmt == object::MOVEMENT_TYPE::STATIC) {
			static_segments.push_back(s);
		}
		else {
			dynamic_segments.push_back(s);
		}
	}
}

redirection_unit apf_vec::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	std::deque<proximity_container*> prox_queue = egocentric_user->proximity_queue;
	vec2f phys_heading = rad_2_vec(egocentric_user->state.get_phys_heading());
	trajectory_unit cur_move = egocentric_user->state.get_cur_move();
	std::vector<user*> other_users = sim_state.get_other_users(egocentric_user);

	force_vector = compute_force_vector(phys_pos, phys_heading, other_users);
	steering_rate = compute_steering_rate(prox_queue);
	redirection_unit next_redirection = set_gains(cur_move, phys_heading, force_vector, steering_rate);
	return next_redirection;
}

vec2f apf_vec::compute_force_vector(vec2f phys_pos, vec2f phys_heading, std::vector<user*> other_users) {
	vec2f f = vec2f(0.0f, 0.0f);

	// Compute forces from obstacles
	for (segment_container s : static_segments) {
		vec2f seg_to_user = phys_pos - s.center;
		float dist = length(seg_to_user);
		// If the user segment is facing the user, contribute force.
		if (dot(s.normal, seg_to_user) > 0.0f) {
			f += FORCE_DISTANCE_SCALE * s.seg_length *
				 normalize(seg_to_user) * (1 / pow(dist, OBSTACLE_FALLOFF_FACTOR));
		}
	}

	// Compute forces from other users
	for (user* u : other_users) {
		vec2f other_phys_pos = u->state.get_phys_pos();
		vec2f other_phys_heading = rad_2_vec(u->state.get_phys_heading());
		vec2f user_to_user = normalize(phys_pos - other_phys_pos);
		float dist = length(phys_pos - other_phys_pos);
		// The paper doesn't say if they use the signed or unsigned angle
		float theta1 = angle(phys_heading, user_to_user);
		float theta2 = angle(other_phys_heading, user_to_user);
		float k = math::clamp((math::cos(theta1) + math::cos(theta2)) * 0.5f, 0.0f, 1.0f);
		f += k * user_to_user * (1 / pow(dist, USER_FALLOFF_FACTOR));
	}

	return f;
}

float apf_vec::compute_steering_rate(std::deque<proximity_container*> prox_queue) {
	float closest_object_distance = prox_queue.front()->get_distance();
	float walking_steering_rate = 360.0f * (config::VELOCITY / (2.0f * math::pi * TURN_ARC_RADIUS));

	// User is too close to the closest object to be passively steered.
	// Thus, we increase the steering rate up to the max rate to help the
	// user avoid the nearby object.
	if (closest_object_distance < TURN_ARC_RADIUS) {
		float t = 1 - (closest_object_distance / TURN_ARC_RADIUS);
		walking_steering_rate = ((1 - t) * walking_steering_rate) + (t * MAX_STEERING_RATE);
	}
	return walking_steering_rate;
}

redirection_unit apf_vec::set_gains(trajectory_unit cur_move, vec2f phys_heading, vec2f force_vector, float steering_rate) {
	redirection_unit redir_unit = redirection_unit();
	/*
	int turn_dir = math::sign(signed_angle(phys_heading, normalize(force_vector)));

	if (cur_move.dtheta) {
		// User has to turn right to face the force vector
		if (turn_dir == -1) {
			// User is turning to the right
			if (cur_move.dtheta < 0) {
				redir_unit.rota_gain = max_rota_gain;
				cur_rota_gain = max_rota_gain;
			}
			// User is turning to the left
			else {
				redir_unit.rota_gain = min_rota_gain;
				cur_rota_gain = min_rota_gain;
			}
		}
		// User has to turn left to face the force vector
		else {
			// User is turning to the right
			if (cur_move.dtheta < 0) {
				redir_unit.rota_gain = min_rota_gain;
				cur_rota_gain = min_rota_gain;
			}
			// User is turning to the left
			else {
				redir_unit.rota_gain = max_rota_gain;
				cur_rota_gain = max_rota_gain;
			}
		}

		redir_unit.apply_rota = true;
		redir_unit.apply_trans = false;
		redir_unit.apply_curve = false;
		redir_unit.apply_bend = false;
	}
	// User is translating
	else if (cur_move.dx || cur_move.dy) {
		redir_unit.curve_gain = steering_rate;
		redir_unit.curve_gain_dir = math::sign(turn_dir);

		// If we should use translation gains (my modification of original paper)
		if (config::APF_USE_TRANS) {
			// User is facing away from the gradient, slow them down
			if (dot(phys_heading, force_vector) < 0) {
				redir_unit.trans_gain = min_trans_gain;
				cur_trans_gain = min_trans_gain;
			}
			else {
				redir_unit.trans_gain = 1.0f;
				cur_trans_gain = 1.0f;
			}
		}
		// No translation gain. Original description of the algorithm.
		else {
			redir_unit.trans_gain = 1.0f; // The original paper doesn't seem to use translation gains.
		}

		redir_unit.apply_rota = false;
		redir_unit.apply_trans = true;
		redir_unit.apply_curve = true;
		redir_unit.apply_bend = false;
	}
	*/
	return redir_unit;
}

std::vector<std::vector<vec2f>> apf_vec::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
	std::vector<std::vector<vec2f>> test;
	return test;
}