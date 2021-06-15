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
		/**
		 * Redirection controller defaul constructor. Don't use this!
		 */
		redirector();

		/**
		 * Constructor that requires a resetter policy. Also, don't use this!
		 */
		redirector(resetter* _resetter);

		/**
		 * In "Rethinking Redirected Walking: On the Use of Curvature Gains Beyond
		 * Perceptual Limitations and Revisiting Bending Gains" by Rietzler et al., 
		 * it is argued that curvature should be reported in degrees rotated per 
		 * meter (deg/m). Most people have reported curvature as the radius of the 
		 * circle required to allow people to walk an infinite straight line given 
		 * their perceptual thresholds. This method converts between the two
		 * notations.
		 *
		 * First calculate the circumference of the circle that would be required
		 * given the circle radius as reported in various research. Then, divide 360 
		 * by this circumference to get the amount of degrees turned for each meter 
		 * along the circumference.
		 */
		float curve_radius_to_deg_per_meter();

		/**
		 * Set the redirection gains according to the user's current position
		 * in the physical and virtual environments.
		 * @param dx The amount the user has moved in the x-direction (lateral) since 
					 the last frame. Positive value is rightward, negative is leftward.
		 * @param dy The amount the user has moved in the y-direction (forward) since 
					 the last frame. Positive value is forward, negative is backwards.
		 * @param dtheta The amount the user has turned since the last frame. 
						 Positive value is turning to the left, negative is to the right.
		 * @param sim_state The state of the simulation on the current frame.
		 * @param egocentric_user The user whom we are computing redirection gains for.
		 * @return The gains to apply on the current frame.
		 */
		virtual redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* user) = 0;
		
		/**
		 * Reset (reorient) the user. This is usually called when the user gets too
		 * close to an obstacle in the physical environment.
		 * @param sim_state The state of the simulation on the current frame.
		 * @param egocentric_user The user whom we are computing redirection gains for.
		 */
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