#include <iostream>

#include <fstream>

#include "arc_simple.h"
#include "math.hpp"
#include "user.hpp"
#include "geometry.h"
#include "config.h"

#include "visibility.hpp"
#include "vector2.hpp"

arc_simple::arc_simple(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter) {
	name = "Simple Alignment RDW";

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

	for (int i = 0; i < SAMPLE_RATE; i++) {
		sample_directions.push_back(((2 * math::pi) / SAMPLE_RATE) * i);
	}

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

redirection_unit arc_simple::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	if (LOWER_FREQUENCY) { // Don't compute gains on every frame
		calculation_freq_counter -= timestep::dt;

		if (calculation_freq_counter <= 0.0f) {
			update_losses(sim_state, egocentric_user);
			cur_alignment = std::vector<float>{ cur_north_loss, cur_east_loss, cur_west_loss };

			redirection_unit next_redirection = set_gains(dx, dy, dtheta, sim_state, egocentric_user);

			prev_loss = cur_loss;
			calculation_freq_counter = CALCULATION_FREQUENCY;
			prev_redirection = next_redirection;

			return next_redirection;
		}
		else {
			return prev_redirection;
		}
	}
	else { // The normal method: compute every frame
		update_losses(sim_state, egocentric_user);
		cur_alignment = std::vector<float>{ cur_north_loss, cur_east_loss, cur_west_loss };

		redirection_unit next_redirection = set_gains(dx, dy, dtheta, sim_state, egocentric_user);

		prev_loss = cur_loss;

		return next_redirection;
	}
}

void arc_simple::update_losses(simulation_state& sim_state, user* egocentric_user) {
	vec2f heading = rad_2_vec(egocentric_user->state.get_phys_heading());
	std::vector<vec2f> directions{ heading,						 // Forward
								   vec2f(heading.y, -heading.x), // Left
		                           vec2f(-heading.y, heading.x)  // Right
	};

	float phys_distance_north, phys_distance_east, phys_distance_west;
	environment* phys_env = (environment*)egocentric_user->physical_env();
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	for (int i = 0; i < directions.size(); i++) {
		float closest = math::max_float;
		vec2f* p1;
		vec2f* p2;
		for (wall* w : phys_env->get_walls()) {
			p1 = w->get_vertices()[0];
			p2 = w->get_vertices()[1];
			float t = geom::ray_line_intersect(&phys_pos, &(directions[i]), p1, p2);
			if (t >= 0 && t < closest) {
				closest = t;
			}
		}
		for (obstacle* o : phys_env->get_obstacles()) {
			for (wall* w : o->get_walls()) {
				p1 = w->get_vertices()[0];
				p2 = w->get_vertices()[1];
				float t = geom::ray_line_intersect(&phys_pos, &(directions[i]), p1, p2);
				if (t >= 0 && t < closest) {
					closest = t;
				}
			}
		}

		// Assign the distance value to the correct direction.
		switch (i) {
		case 0: {
			phys_distance_north = closest;
			break;
		}
		case 1: {
			phys_distance_east = closest;
			break;
		}
		case 2: {
			phys_distance_west = closest;
			break;
		}
		}
	}

	float virt_distance_north, virt_distance_east, virt_distance_west;
	environment* virt_env = (environment*)egocentric_user->virtual_env();
	vec2f virt_pos = egocentric_user->state.get_virt_pos();
	for (int i = 0; i < directions.size(); i++) {
		float closest = math::max_float;
		vec2f* p1;
		vec2f* p2;
		for (wall* w : virt_env->get_walls()) {
			p1 = w->get_vertices()[0];
			p2 = w->get_vertices()[1];
			float t = geom::ray_line_intersect(&virt_pos, &(directions[i]), p1, p2);
			if (t >= 0 && t < closest) {
				closest = t;
			}
		}
		for (obstacle* o : virt_env->get_obstacles()) {
			for (wall* w : o->get_walls()) {
				p1 = w->get_vertices()[0];
				p2 = w->get_vertices()[1];
				float t = geom::ray_line_intersect(&virt_pos, &(directions[i]), p1, p2);
				if (t >= 0 && t < closest) {
					closest = t;
				}
			}
		}

		// Assign the distance value to the correct direction.
		switch (i) {
		case 0: {
			virt_distance_north = closest;
			break;
		}
		case 1: {
			virt_distance_east = closest;
			break;
		}
		case 2: {
			virt_distance_west = closest;
			break;
		}
		}
	}

	cur_north_loss = phys_distance_north - virt_distance_north;
	cur_east_loss = phys_distance_east - virt_distance_east;
	cur_west_loss = phys_distance_west - virt_distance_west;
}

redirection_unit arc_simple::set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	redirection_unit redir_unit;

	cur_loss = math::abs(cur_north_loss) + math::abs(cur_east_loss) + math::abs(cur_west_loss);
	if (cur_loss < REDIRECTION_LOSS_THRESHOLD ||
		timestep::num_timesteps == 0) {
		cur_rota_gain = 1.0f;
		cur_trans_gain = 1.0f;
		cur_curve_gain = 0.0f;
		apply_rota = false;
		apply_trans = false;
		apply_curve = false;
		return redir_unit;
	}

	if (dtheta) {
		float loss_difference = cur_loss - prev_loss;
		// User is turning in a way that makes the alignment worse. Slow down their turning.
		if (cur_loss > prev_loss) {
			redir_unit.rota_gain = ((1.0f - ROTA_GAIN_SMOOTHING) * prev_rota_gain) + (min_rota_gain * ROTA_GAIN_SMOOTHING);
			//redir_unit.rota_gain = 1.0f;
		}
		// User is turning in a way that improves the alignment. Speed up their turning.
		else {
			redir_unit.rota_gain = ((1.0f - ROTA_GAIN_SMOOTHING) * prev_rota_gain) + (max_rota_gain * ROTA_GAIN_SMOOTHING);
			//redir_unit.rota_gain = 1.0f;
		}

		redir_unit.apply_rota = true;
		redir_unit.apply_trans = false;
		redir_unit.apply_curve = false;
		redir_unit.apply_bend = false;
	}
	else if (dx || dy) {
		// The space to the left of the user is more misaligned than the 
		// space to the right of the user. Turn them to the left.
		if (cur_west_loss > cur_east_loss) {
			//redir_unit.curve_gain = cur_curve_per_deg;
			redir_unit.curve_gain = set_curvature_gain(CURVE_TO_LEFT);
			redir_unit.curve_gain_dir = CURVE_TO_LEFT;
			redir_unit.apply_curve = true;
		}
		// The space to the right of the user is more misaligned than the 
		// space to the right of the user. Turn them to the right.
		else if (cur_east_loss > cur_west_loss) {
			//redir_unit.curve_gain = cur_curve_per_deg;
			redir_unit.curve_gain = set_curvature_gain(CURVE_TO_RIGHT);
			redir_unit.curve_gain_dir = CURVE_TO_RIGHT;
			redir_unit.apply_curve = true;
		}
		// The spaces on both sides of the user are equally aligned, so don't turn them.
		else {
			redir_unit.apply_curve = false;
		}

		// There is more physical space in front of the user than there is virtual 
		// space in front of the user, so make them walk faster in real world.
		if (cur_north_loss > 0.0f) {
			//redir_unit.trans_gain = max_trans_gain;
			redir_unit.trans_gain = set_translation_gain(sim_state, egocentric_user);
			redir_unit.apply_trans = true;
		}
		else if (cur_north_loss < 0.0f) {
			//redir_unit.trans_gain = min_trans_gain;
			redir_unit.trans_gain = set_translation_gain(sim_state, egocentric_user);
			redir_unit.apply_trans = true;
		}
		else {
			redir_unit.trans_gain = 1.0f;
			redir_unit.apply_trans = false;
		}

		redir_unit.apply_rota = false;
		redir_unit.apply_bend = false;
	}

	apply_rota = redir_unit.apply_rota;
	cur_rota_gain = redir_unit.rota_gain;
	apply_trans = redir_unit.apply_trans;
	cur_trans_gain = redir_unit.trans_gain;
	apply_curve = redir_unit.apply_curve;
	cur_curve_gain = redir_unit.curve_gain;
	curve_dir = redir_unit.curve_gain_dir;
	return redir_unit;
}

float arc_simple::set_translation_gain(simulation_state& sim_state, user* egocentric_user) {
	float virt_distance_north = get_distance_in_direction(egocentric_user->state.get_virt_pos(), rad_2_vec(egocentric_user->state.get_virt_heading()), (environment*)(sim_state.virt_env));
	float phys_distance_north = get_distance_in_direction(egocentric_user->state.get_phys_pos(), rad_2_vec(egocentric_user->state.get_phys_heading()), (environment*)(egocentric_user->get_phys_env()));

	float trans_gain = phys_distance_north / virt_distance_north;
	trans_gain = math::clamp(trans_gain, min_trans_gain, max_trans_gain);

	return trans_gain;
}

float arc_simple::set_curvature_gain(int direction) {
	float scale;

	if (direction == CURVE_TO_LEFT) {
		scale = math::min(1.0f, cur_west_loss);
	}
	else {
		scale = math::min(1.0f, cur_east_loss);
	}

	return math::min(cur_curve_per_deg * math::abs(scale), cur_curve_per_deg);
}

/**
 * Dampen the gain according to how close the user is to facing the target direction.
 * Introduced in "Redirected walking to explore virtual environments: Assessing the
 * potential for spatial interference".
 */
float arc_simple::attenuate_gain(float angle_to_target, float cur_gain) {
	float angle = math::degrees(math::abs(angle_to_target));
	if (angle >= 45.0f) return cur_gain;
	else {
		float f = cur_gain * (angle / 45.0f);
		return cur_gain * (angle / 45.0f);
	}
}

float arc_simple::get_distance_in_direction(vec2f pos, vec2f dir, environment* env) {
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

float arc_simple::distance_to_closest_feature(vec2f pos, vec2f dir, visibility_polygon vis_poly) {
	float min_dist = math::max_float;
	for (int i = 0; i < vis_poly.verts.size(); i++) {
		vec2f* p1 = vis_poly.verts[i % vis_poly.verts.size()];
		vec2f* p2 = vis_poly.verts[(i + 1) % vis_poly.verts.size()];
		float t = geom::ray_line_intersect(&pos, &dir, p1, p2);
		if (t >= 0.0f && t < min_dist) {
			min_dist = t;
		}
	}
	return min_dist;
}

vec2f arc_simple::get_optimal_heading(vec2f pos, visibility_polygon vis_poly, float dist_north, float dist_east, float dist_south, float dist_west) {
	float best_distance_loss = math::max_float;
	vec2f best_dir;
	for (float dir_theta : sample_directions) {
		vec2f dir = rad_2_vec(dir_theta);
		float distance_north = distance_to_closest_feature(pos, dir, vis_poly);
		float distance_east = distance_to_closest_feature(pos, vec2f(dir.y, -dir.x), vis_poly);
		float distance_south = distance_to_closest_feature(pos, vec2f(-dir.x, -dir.y), vis_poly);
		float distance_west = distance_to_closest_feature(pos, vec2f(-dir.y, dir.x), vis_poly);
		float sum = 0.0f;
		sum += math::abs(distance_north - dist_north) * 50.0f;
		sum += math::abs(distance_east - dist_east);
		sum += math::abs(distance_south - dist_south);
		sum += math::abs(distance_west - dist_west);
		if (sum < best_distance_loss) {
			best_distance_loss = sum;
			best_dir = dir;
		}
	}

	return best_dir;
}

std::vector<std::vector<vec2f>> arc_simple::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
	std::vector<std::vector<vec2f>> test;
	return test;
}

void arc_simple::reset_to_gradient(vec2f phys_heading, float virt_heading, std::vector<trajectory_unit>* path, std::deque<proximity_container*> prox_queue, physical_environment* phys_env, vec2f phys_pos, vec2f virt_pos) {
	gradient_dir = compute_gradient(phys_env, prox_queue, phys_pos);
	reorient_to_target(phys_heading, virt_heading, phys_pos, virt_pos, gradient_dir + phys_pos, path);
}

vec2f arc_simple::compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos) {
	std::vector<object*> objects;
	for (proximity_container* c : prox_queue) {
		if (c->get_obstacle()->space == object::SPACE_TYPE::PHYS) {
			objects.push_back(c->get_obstacle());
		}
	}
	float cur_repulsive_force = compute_repulsive_force(phys_pos, objects);

	float min_force = std::numeric_limits<float>::max();
	vec2f min_force_pos;
	for (float theta : theta_values) {
		vec2f new_pos = phys_pos + (rad_2_vec(theta) * SFR2G_STEP_SIZE);
		if (!phys_env->point_is_legal(new_pos)) continue;

		float new_force = compute_repulsive_force(new_pos, objects);
		if (new_force <= min_force) {
			min_force = new_force;
			min_force_pos = new_pos;
		}
	}

	assert(min_force != std::numeric_limits<float>::max());
	return normalize(min_force_pos - phys_pos);
}

float arc_simple::compute_repulsive_force(vec2f pos, std::vector<object*> objects) {
	float dist_sum = 0.0f;
	for (object* obj : objects) {
		if (obj->space == object::SPACE_TYPE::PHYS)
			dist_sum += 1.0f / obj->distance(pos);
	}
	return dist_sum;
}