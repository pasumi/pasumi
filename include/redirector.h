#pragma once

#include <deque>

// Forward declare
class user;

#include "math.hpp"
#include "physical_environment.h"
#include "proximity_container.h"
#include "vec2f.h"
#include "motion_model.h"
#include "timestep.h"
#include "simulation_state.h"
#include "resetter.h"

struct redirection_unit {
	redirection_unit() {
		apply_rota = false;
		apply_trans = false;
		apply_curve = false;
		apply_bend = false;
		rota_gain = 1.0f;
		trans_gain = 1.0f;
		curve_gain = 0.0f;
		curve_gain_dir = 1;
		bend_gain = 0.0f;
	}

	bool apply_rota;
	bool apply_trans;
	bool apply_curve;
	bool apply_bend;
	float rota_gain;
	float trans_gain;
	float curve_gain;
	int curve_gain_dir;
	float bend_gain;
};

class redirector {
	public:
		redirector();
		redirector(resetter* _resetter);
		float curve_radius_to_deg_per_meter();
		virtual redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* user) = 0;
		void reset(simulation_state& sim_state, user* user);

		char* name;
		int reset_timer; // Number of timesteps it will take to complete the reset
		int post_reset_timer;
		redirection_unit resetting_gains; // Gains to use during resetting

		bool apply_rota;
		float cur_rota_gain;
		float min_rota_gain;
		float max_rota_gain;
		int rota_dir;

		bool apply_trans;
		float cur_trans_gain;
		float min_trans_gain;
		float max_trans_gain;

		bool apply_curve;
		float cur_curve_gain;
		float curve_radius;
		float cur_curve_per_deg;
		int curve_dir; // curve to the left of the user == 1. curve to the right of the user == -1
		const int CURVE_TO_LEFT = 1;
		const int CURVE_TO_RIGHT = -1;

		float cur_bend_gain;
		float min_bend_gain;
		float max_bend_gain;

		resetter* reset_policy;

	protected:
		bool check_redirection(redirection_unit unit);
};