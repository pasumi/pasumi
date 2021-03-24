#pragma once

#include "redirector.h"

class arc : public redirector {
public:
	arc(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter);
	redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);

	std::vector<float> sample_directions;

private:
	void update_losses(simulation_state& sim_state, user* egocentric_user);
	redirection_unit set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	float set_translation_gain(simulation_state& sim_state, user* egocentric_user);
	float set_curvature_gain(int direction);

	const int SAMPLE_RATE = 20;
	const float REDIRECTION_LOSS_THRESHOLD = 0.1f;
	const float ROTA_GAIN_SMOOTHING = 0.125f;

	std::vector<float> cur_alignment;
	float prev_loss;
	float cur_loss;
	float cur_north_loss;
	float cur_east_loss;
	float cur_west_loss;
	float prev_rota_gain;
};