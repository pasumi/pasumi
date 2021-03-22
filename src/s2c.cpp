#include "s2c.h"

s2c::s2c(resetter* _resetter) {
	name = "S2C RDW";

	cur_rota_gain = 1.0f;
	min_rota_gain = 0.81f;
	max_rota_gain = 1.35f;

	cur_trans_gain = 1.0f;
	min_trans_gain = 0.86f;
	max_trans_gain = 1.26f;

	curve_radius = 7.5f; // meters
	cur_curve_per_deg = curve_radius_to_deg_per_meter();
	curve_dir = 1; // curve to the left of the user == 1. curve to the right of the user == -1

	reset_policy = _resetter;
}

redirection_unit s2c::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	redirection_unit redir_unit = set_gains(dx, dy, dtheta, egocentric_user->state.get_phys_heading(), egocentric_user->state.get_phys_pos());
	return redir_unit;
}

redirection_unit s2c::set_gains(float dx, float dy, float dtheta, float phys_heading, vec2f phys_pos) {
	redirection_unit redir_unit = redirection_unit();
	vec2f user_heading = rad_2_vec(phys_heading);
	float angle_to_center = signed_angle(user_heading, normalize(center - phys_pos));

	if (math::abs(angle_to_center) > math::radians(160.0f)) {
		// Generate target to the right side of the user
		if (angle_to_center < 0.0f) {
			vec2f dir = vec2f(user_heading.y, -user_heading.x);
			dir = dir * 5.0f;
			target = phys_pos + dir;
		}
		// Generate target to the left side of the user
		else {
			vec2f dir = vec2f(-user_heading.y, user_heading.x);
			dir = dir * 5.0f;
			target = phys_pos + dir;
		}
	}
	else {
		target = center;
	}

	float angle_to_target = signed_angle(user_heading, normalize(target - phys_pos));
	float distance_to_target = length(phys_pos - target);

    if (dtheta) {
        // User has to turn right to face the center
        if (math::sign(angle_to_target) == -1) {
			// User is turning to the right
			if (dtheta < 0) {
				redir_unit.rota_gain = max_rota_gain;
				cur_rota_gain = max_rota_gain;
			}
			// User is turning to the left
			else {
				redir_unit.rota_gain = min_rota_gain;
				cur_rota_gain = min_rota_gain;
			}
		}
		// User has to turn left to face the center
		else {
			// User is turning to the right
			if (dtheta < 0) {
				redir_unit.rota_gain = min_rota_gain;
				cur_rota_gain = min_rota_gain;
			}
			// User is turning to the left
			else {
				redir_unit.rota_gain = max_rota_gain;
				cur_rota_gain = max_rota_gain;
			}
		}
		redir_unit.curve_gain = attenuate_rotation_gain(angle_to_target, distance_to_target, redir_unit.curve_gain);

		redir_unit.apply_rota = true;
		redir_unit.apply_trans = false;
		redir_unit.apply_curve = false;
		redir_unit.apply_bend = false;
	}
	else if (dx || dy) {
		// User has to turn right to face the center
		if (math::sign(angle_to_target) == -1) {
			redir_unit.curve_gain = TRANSLATIONAL_ROTATION;
			redir_unit.curve_gain_dir = -1;
			curve_dir = -1;
		}
		// User has to turn left to face the center
		else {
			redir_unit.curve_gain = TRANSLATIONAL_ROTATION;
			redir_unit.curve_gain_dir = 1;
			curve_dir = 1;
		}
		redir_unit.curve_gain = attenuate_curvature_gain(angle_to_target, distance_to_target, redir_unit.curve_gain);

		redir_unit.trans_gain = 1.0f;

		redir_unit.apply_rota = false;
		redir_unit.apply_trans = false;
		redir_unit.apply_curve = true;
		redir_unit.apply_bend = false;
	}
    
	return redir_unit;
}

float s2c::attenuate_rotation_gain(float angle_to_target, float distance_to_center, float rota_gain) {
	float heading_dampen = math::min(math::abs(angle_to_target / (math::pi / 4.0f)), 1.0f);
	float distance_dampen = math::min(math::abs(distance_to_center / DISTANCE_DAMPING_THRESHOLD), 1.0f);
	float dampen = (heading_dampen + distance_dampen) * 0.5f;
	rota_gain *= dampen;
	return rota_gain;
}

float s2c::attenuate_curvature_gain(float angle_to_target, float distance_to_center, float curve_gain) {
	float heading_dampen; 
	if (math::abs(angle_to_target) < math::pi / 4.0f) {
		heading_dampen  = math::abs(angle_to_target / (math::pi / 4.0f));
	}
	else {
		heading_dampen = 1.0f;
	}

	float distance_dampen;
	if (distance_to_center < DISTANCE_DAMPING_THRESHOLD) {
		distance_dampen = math::abs(distance_to_center / DISTANCE_DAMPING_THRESHOLD);
	}
	else {
		distance_dampen = 1.0f;
	}

	float dampen = (heading_dampen + distance_dampen) * 0.5f;
	curve_gain *= dampen;
	return curve_gain;
}

std::vector<std::vector<vec2f>> s2c::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
	std::vector<std::vector<vec2f>> test;
	return test;
}