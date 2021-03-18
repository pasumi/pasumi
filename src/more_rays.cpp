#include <iostream>

#include "more_rays.h"
#include "math.hpp"
#include "user.hpp"
#include "geometry.h"
#include "config.h"
#include "vector2.hpp"

more_rays::more_rays() {

}

more_rays::more_rays(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter) {
	name = "More rays ARC RDW";

	cur_rota_gain = 1.0f;
	min_rota_gain = 0.67f;
	max_rota_gain = 1.24f;
	prev_rota_gain = 1.0f;

	cur_trans_gain = 1.0f;
	min_trans_gain = 0.86f;
	max_trans_gain = 1.26f;

	curve_radius = 7.5f; // meters
	cur_curve_per_deg = curve_radius_to_deg_per_meter();
	curve_dir = 1; // curve to the left of the user == 1. curve to the right of the user == -1

	prev_loss = -1.0f;
	cur_loss = -1.0f;

	cur_alignment = std::vector<float>{ 0.0f, 0.0f, 0.0f };

	reset_policy = _resetter;

	calculation_freq_counter = CALCULATION_FREQUENCY;

	//============================
	// Gradient stuff
	for (int i = 0; i < GRADIENT_SAMPLE_RATE; i++) {
		theta_values.push_back(((2 * math::pi) / GRADIENT_SAMPLE_RATE) * i);
	}
}

more_rays::~more_rays() {

}

redirection_unit more_rays::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	float phys_heading = egocentric_user->state.get_phys_heading();
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	float virt_heading = egocentric_user->state.get_virt_heading();
	vec2f virt_pos = egocentric_user->state.get_virt_pos();
	vec2f force_dir = compute_force_dir(sim_state, egocentric_user);

	return set_gains(dx, dy, dtheta, sim_state, egocentric_user, force_dir);
}

redirection_unit more_rays::set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user, vec2f force_dir) {
	redirection_unit redir_unit;
	timestep::num_timesteps;

	// No redirection
	if (length(force_dir) <= 0.01f) {
		redir_unit.apply_rota = false;
		redir_unit.apply_trans = false;
		redir_unit.apply_curve = false;
		redir_unit.apply_bend = false;
		return redir_unit;
	}

	float phys_heading = egocentric_user->state.get_phys_heading();
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	float virt_heading = egocentric_user->state.get_virt_heading();
	vec2f virt_pos = egocentric_user->state.get_virt_pos();
	float amount_to_rotate_force_dir = (math::pi * 0.5f) - phys_heading;
	vec2f rotated_force_dir = rotate_around(vec2f(0.0f, 0.0f), force_dir, amount_to_rotate_force_dir);
	rotated_force_dir += phys_pos;

	float force_dir_mag = length(rotated_force_dir); // strength of gain
	float force_dir_height = phys_pos.y - rotated_force_dir.y; // trans gain
	float force_dir_width = phys_pos.x - rotated_force_dir.x; // curvature gain

	vec2f user_dir = rad_2_vec(phys_heading);
	float angle_to_force_dir = signed_angle(user_dir, normalize(force_dir));

	if (dtheta) {
		// User has to turn right to face the gradient
		if (math::sign(angle_to_force_dir) == -1) {
			// User is turning to the right
			if (dtheta < 0) {
				//float max_increased = max_rota_gain * dtheta;
				//float scale = math::min(1.0f, math::abs(max_increased) / math::abs(angle_to_force_dir));
				//redir_unit.rota_gain = max_rota_gain * scale;
				redir_unit.rota_gain = max_rota_gain;
				cur_rota_gain = redir_unit.rota_gain;
			}
			// User is turning to the left
			else {
				redir_unit.rota_gain = min_rota_gain;
				cur_rota_gain = redir_unit.rota_gain;
			}
		}
		// User has to turn left to face the gradient
		else {
			// User is turning to the right
			if (dtheta < 0) {
				redir_unit.rota_gain = min_rota_gain;
				cur_rota_gain = redir_unit.rota_gain;
			}
			// User is turning to the left
			else {
				//float max_increased = max_rota_gain * dtheta;
				//float scale = math::min(1.0f, math::abs(max_increased) / math::abs(angle_to_force_dir));
				//redir_unit.rota_gain = max_rota_gain * scale;

				redir_unit.rota_gain = max_rota_gain;
				cur_rota_gain = redir_unit.rota_gain;
			}
		}

		redir_unit.apply_rota = true;
		redir_unit.apply_trans = false;
		redir_unit.apply_curve = false;
		redir_unit.apply_bend = false;
	}
	// User is translating
	else if (dx || dy) {
		// User has to turn right to face the gradient
		if (math::sign(angle_to_force_dir) == -1) {
			redir_unit.curve_gain = cur_curve_per_deg;
			//redir_unit.curve_gain = set_curvature_gain(force_dir_width, angle_to_force_dir);
				redir_unit.curve_gain_dir = CURVE_TO_RIGHT;
			redir_unit.apply_curve = true;
		}
		// User has to turn left to face the gradient
		else if (math::sign(angle_to_force_dir) == 1) {
			redir_unit.curve_gain = cur_curve_per_deg;
			//redir_unit.curve_gain = set_curvature_gain(force_dir_width, angle_to_force_dir);
			redir_unit.curve_gain_dir = CURVE_TO_LEFT;
			redir_unit.apply_curve = true;
		}
		// The spaces on both sides of the user are equally aligned, so don't turn them.
		else {
			redir_unit.apply_curve = false;
		}

		redir_unit.trans_gain = set_translation_gain(user_dir, force_dir);
		redir_unit.apply_trans = true;

		//// There is more physical space in front of the user than there is virtual 
		//// space in front of the user, so make them walk faster in real world.
		//if (force_dir_height > 0.0f) {
		//	//redir_unit.trans_gain = max_trans_gain;
		//	redir_unit.trans_gain = set_translation_gain(user_dir, force_dir);
		//	redir_unit.apply_trans = true;
		//}
		//else if (force_dir_height < 0.0f) {
		//	//redir_unit.trans_gain = min_trans_gain;
		//	redir_unit.trans_gain = set_translation_gain(user_dir, force_dir);
		//	redir_unit.apply_trans = true;
		//}
		//else {
		//	redir_unit.trans_gain = 1.0f;
		//	redir_unit.apply_trans = false;
		//}

		redir_unit.apply_rota = false;
		redir_unit.apply_bend = false;
	}

	return redir_unit;
}

float more_rays::set_curvature_gain(float misalignment, float angle_to_force_dir) {
	float scale = math::min(1.0f, math::abs(angle_to_force_dir) / math::radians(cur_curve_per_deg));
	return scale * cur_curve_per_deg;
}

float more_rays::set_translation_gain(vec2f user_dir, vec2f force_dir) {
	float l = length(force_dir);
	// User is facing away, slow them down
	if (dot(user_dir, force_dir) < 0) {
		float diff = 1.0f - min_trans_gain;
		return min_trans_gain;
		return math::max(min_trans_gain, 1.0f - (diff * l));
	}
	else {
		return max_trans_gain;
		return math::min(max_trans_gain, max_trans_gain * l);
	}
}

vec2f more_rays::compute_force_dir(simulation_state& sim_state, user* egocentric_user) {
	float phys_heading = egocentric_user->state.get_phys_heading();
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	float virt_heading = egocentric_user->state.get_virt_heading();
	vec2f virt_pos = egocentric_user->state.get_virt_pos();
	vec2f force_dir = vec2f(0.0f, 0.0f);
	auto ft1 = vec2f(0.0f, 0.0f);
	auto ft2 = vec2f(0.0f, 0.0f);
	auto weighted_vec = vec2f(0.0f, 0.0f);

	std::vector<vec2f> phys_dirs = compute_rays(phys_heading);
	std::vector<vec2f> virt_dirs = compute_rays(virt_heading);
	for (int i = 0; i < phys_dirs.size(); i++) {
		vec2f v = get_steering_vector(sim_state, egocentric_user, phys_dirs[i], virt_dirs[i]);
		weighted_vec += normalize(v) * length(v) * (1.0f / phys_dirs.size());
		ft1 += v / phys_dirs.size();
		ft2 += v;
		//force_dir += get_steering_vector(sim_state, egocentric_user, phys_dirs[i], virt_dirs[i]) * (1.0f / phys_dirs.size());
		//force_dir += get_steering_vector(sim_state, egocentric_user, phys_dirs[i], virt_dirs[i]);
	}

	return weighted_vec;
	return ft1;
	return force_dir;
}

vec2f more_rays::get_steering_vector(simulation_state& sim_state, user* egocentric_user, vec2f phys_dir, vec2f virt_dir) {
	float phys_heading = egocentric_user->state.get_phys_heading();
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	float virt_heading = egocentric_user->state.get_virt_heading();
	vec2f virt_pos = egocentric_user->state.get_virt_pos();

	float phys_distance = get_distance_in_direction(phys_pos, phys_dir, egocentric_user->physical_env());
	float virt_distance = get_distance_in_direction(virt_pos, virt_dir, sim_state.virt_env);
	vec2f result = (phys_dir * phys_distance) - (virt_dir * virt_distance);
	vec2f result2 = phys_dir * (phys_distance-virt_distance);
	return result;
}

std::vector<vec2f> more_rays::compute_rays(float heading) {
	std::vector<vec2f> dirs;
	float start_theta = math::pi * 0.5f;
	int num_rays = 89;
	float theta_increment = math::two_pi / num_rays;
	for (int i = 0; i < num_rays; i++) {
		dirs.push_back(rad_2_vec(heading + (theta_increment * i)));
	}
	return dirs;
}

float more_rays::get_distance_in_direction(vec2f pos, vec2f dir, environment* env) {
	float closest = math::max_float;

	vec2f* p1;
	vec2f* p2;
	for (wall* w : env->get_walls()) {
		p1 = w->get_vertices()[0];
		p2 = w->get_vertices()[1];
		float t = geom::ray_line_intersect(&pos, &dir, p1, p2);
		if (t >= 0 && t < closest) {
			closest = t;
		}
	}
	for (obstacle* o : env->get_obstacles()) {
		for (wall* w : o->get_walls()) {
			p1 = w->get_vertices()[0];
			p2 = w->get_vertices()[1];
			float t = geom::ray_line_intersect(&pos, &dir, p1, p2);
			if (t >= 0 && t < closest) {
				closest = t;
			}
		}
	}

	return closest;
}

std::vector<std::vector<vec2f>> more_rays::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
	std::vector<std::vector<vec2f>> test;
	return test;
}