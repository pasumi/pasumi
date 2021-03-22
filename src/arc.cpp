#include <iostream>

#include <fstream>

#include "arc.h"
#include "math.hpp"
#include "user.hpp"
#include "geometry.h"
#include "config.h"
#include "vector2.hpp"

arc::arc(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter) {
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
}

redirection_unit arc::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	update_losses(sim_state, egocentric_user);
	cur_alignment = std::vector<float>{ cur_north_loss, cur_east_loss, cur_west_loss };

	redirection_unit next_redirection = set_gains(dx, dy, dtheta, sim_state, egocentric_user);

	prev_loss = cur_loss;

	return next_redirection;
}

void arc::update_losses(simulation_state& sim_state, user* egocentric_user) {
	vec2f phys_heading = rad_2_vec(egocentric_user->state.get_phys_heading());
	std::vector<vec2f> directions{ phys_heading, // Forward (north)
		vec2f(phys_heading.y, -phys_heading.x), // Right (east)
		vec2f(-phys_heading.y, phys_heading.x)  // Left (west)
	};

	environment* phys_env = (environment*)egocentric_user->physical_env();
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	float phys_distance_north = phys_env->get_distance_in_direction_from_point(phys_pos, directions[0]);
	float phys_distance_east = phys_env->get_distance_in_direction_from_point(phys_pos, directions[1]);
	float phys_distance_west = phys_env->get_distance_in_direction_from_point(phys_pos, directions[2]);

	vec2f virt_heading = rad_2_vec(egocentric_user->state.get_virt_heading());
	directions = std::vector<vec2f>{ virt_heading, // Forward (north)
		vec2f(virt_heading.y, -virt_heading.x), // Right (east)
		vec2f(-virt_heading.y, virt_heading.x)  // Left (west)
	};
	environment* virt_env = (environment*)egocentric_user->virtual_env();
	vec2f virt_pos = egocentric_user->state.get_virt_pos();
	float virt_distance_north = virt_env->get_distance_in_direction_from_point(virt_pos, directions[0]);
	float virt_distance_east = virt_env->get_distance_in_direction_from_point(virt_pos, directions[1]);
	float virt_distance_west = virt_env->get_distance_in_direction_from_point(virt_pos, directions[2]);

	cur_north_loss = phys_distance_north - virt_distance_north;
	cur_east_loss = phys_distance_east - virt_distance_east;
	cur_west_loss = phys_distance_west - virt_distance_west;
}

redirection_unit arc::set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
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
		}
		// User is turning in a way that improves the alignment. Speed up their turning.
		else {
			redir_unit.rota_gain = ((1.0f - ROTA_GAIN_SMOOTHING) * prev_rota_gain) + (max_rota_gain * ROTA_GAIN_SMOOTHING);
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
			redir_unit.trans_gain = set_translation_gain(sim_state, egocentric_user);
			redir_unit.apply_trans = true;
		}
		else if (cur_north_loss < 0.0f) {
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

float arc::set_translation_gain(simulation_state& sim_state, user* egocentric_user) {
	float virt_distance_north = sim_state.virt_env->get_distance_in_direction_from_point(egocentric_user->state.get_virt_pos(), rad_2_vec(egocentric_user->state.get_virt_heading()));
	float phys_distance_north = egocentric_user->get_phys_env()->get_distance_in_direction_from_point(egocentric_user->state.get_phys_pos(), rad_2_vec(egocentric_user->state.get_phys_heading()));

	float trans_gain = phys_distance_north / virt_distance_north;
	trans_gain = math::clamp(trans_gain, min_trans_gain, max_trans_gain);

	return trans_gain;
}

float arc::set_curvature_gain(int direction) {
	float scale;

	if (direction == CURVE_TO_LEFT) {
		scale = math::min(1.0f, cur_west_loss);
	}
	else {
		scale = math::min(1.0f, cur_east_loss);
	}

	return math::min(cur_curve_per_deg * math::abs(scale), cur_curve_per_deg);
}
