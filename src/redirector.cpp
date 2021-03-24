#include "redirector.h"
#include "config.h"

redirector::redirector() {
	post_reset_timer = 0;

	cur_rota_gain = 1.0f;
	cur_trans_gain = 1.0f;
	cur_curve_per_deg = 0.0f;
}

redirector::redirector(resetter* _resetter) {
	reset_policy = _resetter;
	post_reset_timer = 0;

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