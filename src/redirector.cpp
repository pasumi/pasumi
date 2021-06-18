#include "redirector.h"
#include "config.h"

redirector::redirector() {
	cur_rota_gain = 1.0f;
	cur_trans_gain = 1.0f;
	cur_curve_per_deg = 0.0f;
}

redirector::redirector(resetter* _resetter) {
	reset_policy = _resetter;
	cur_rota_gain = 1.0f;
	cur_trans_gain = 1.0f;
	cur_curve_per_deg = 0.0f;
}

float redirector::curve_radius_to_deg_per_meter() {
	return 360.0f / (2.0f * math::pi * curve_radius);
}
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