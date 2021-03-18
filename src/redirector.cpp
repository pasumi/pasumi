#include "redirector.h"
#include "config.h"

redirector::redirector() {
	post_reset_timer = 0;
	POST_RESET_GRACE_PERIOD = config::POST_RESET_GRACE_PERIOD;

	cur_rota_gain = 1.0f;
	cur_trans_gain = 1.0f;
	cur_curve_per_deg = 0.0f;
}

redirector::redirector(resetter* _resetter) {
	reset_policy = _resetter;
	post_reset_timer = 0;
	POST_RESET_GRACE_PERIOD = config::POST_RESET_GRACE_PERIOD;

	cur_rota_gain = 1.0f;
	cur_trans_gain = 1.0f;
	cur_curve_per_deg = 0.0f;
}

/**
 * In "Rethinking Redirected Walking: On the Use of Curvature Gains Beyond
 * Perceptual Limitations and Revisiting Bending Gains" by Rietzler et al., it is
 * argued that curvature should be reported in degrees rotated per meter (deg/m).
 * Most people have reported curvature as the radius of the circle required to allow
 * people to walk an infinite straight line given their perceptual thresholds.
 * This method converts between the two notations.
 *
 * First calculate the circumference of the circle that would be required given the
 * circle radius as reported in various research. 
 * Then, divide 360 by this circumference to get the amount of degrees turned 
 * for each meter along the circumference.
 */
float redirector::curve_radius_to_deg_per_meter() {
	return 360.0f / (2.0f * math::pi * curve_radius);
}

/**
 * Ensures that we apply the correct redirection gains.
 */
bool redirector::check_redirection(redirection_unit unit) {
	return (unit.apply_rota && !unit.apply_curve && !unit.apply_trans) ||
		   (!unit.apply_rota && unit.apply_curve && unit.apply_trans);
}

void redirector::reorient_to_target(vec2f phys_heading, float virt_heading, vec2f phys_pos, vec2f virt_pos, vec2f target, std::vector<trajectory_unit>* path) {
	float angle_per_dt = math::radians(config::ANGULAR_VELOCITY) / (1.0f / timestep::dt);

	float angle_to_target = signed_angle(phys_heading, normalize(target - phys_pos));
	float complement_angle = (math::pi * 2) - math::abs(angle_to_target); // Complement of angle to target
	complement_angle *= -math::sign(angle_to_target);	// Flip the direction

	resetting_gains.apply_rota = true;
	resetting_gains.rota_gain = math::abs(complement_angle) / (math::pi * 2); // Gain needed to have the 360 virtual turn result in a physical turn towards the target (phys env center).
	resetting_gains.apply_trans = false;
	resetting_gains.trans_gain = 1.0f;
	resetting_gains.apply_curve = false;
	resetting_gains.curve_gain = 0.0f;
	resetting_gains.apply_bend = false;
	resetting_gains.bend_gain = 0.0f;

	float cur_heading = virt_heading;

	int num_iter = math::two_pi / angle_per_dt;
	reset_timer = num_iter;
	for (int i = 0; i < num_iter; i++) {
		cur_heading += angle_per_dt * math::sign(complement_angle) * -1;
		cur_heading = math::clamp_rotation(cur_heading);
		path->insert(path->begin(), trajectory_unit(virt_pos.x, virt_pos.y, cur_heading));
	}
	float remainder = math::two_pi - (angle_per_dt * num_iter);
	if (remainder) {
		cur_heading += remainder;
		cur_heading = math::clamp_rotation(cur_heading);
		path->insert(path->begin(), trajectory_unit(virt_pos.x, virt_pos.y, cur_heading + remainder));
	}
}

void redirector::reset(simulation_state& sim_state, user* user) {
	float rotation_gain_for_reset = reset_policy->reset(sim_state, user);
	resetting_gains.apply_rota = true;
	resetting_gains.rota_gain = rotation_gain_for_reset;
	resetting_gains.apply_trans = false;
	resetting_gains.trans_gain = 1.0f;
	resetting_gains.apply_curve = false;
	resetting_gains.curve_gain = 0.0f;
	resetting_gains.apply_bend = false;
	resetting_gains.bend_gain = 0.0f;
}