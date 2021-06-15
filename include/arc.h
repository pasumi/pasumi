#pragma once

#include "redirector.h"

/**
 * An implementation of the ARC redirection controller. This algorithm was
 * first presented in the paper "ARC: Alignment-based Redirection Controller for
 * Redirected Walking in Complex Environments" by Niall L. Williams et al.
 * For more information, see https://gamma.umd.edu/arc/.
 */
class arc : public redirector {
public:
	/**
	 * ARC controller default constructor.
	 */
	arc(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter);

	/**
	 * Computes the redirection gains to apply on the current frame.
	 */
	redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);

	std::vector<float> sample_directions; // Directions to check when resetting.

private:
	/**
	 * Compute the distances to the closest obstacles in each direction,
	 * and then update the differences between these physical and virtual
	 * distances.
	 * @param sim_state The state of the simulation on the current frame.
	 * @param egocentric_user The user whom we are computing redirection gains for.
	 */
	void update_losses(simulation_state& sim_state, user* egocentric_user);

	/**
	 * Set the redirection gains according to the user's current position
	 * in the physical and virtual environments.
	 */
	redirection_unit set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);

	/**
	 * Sets the translation gain according to the ratio between the distances
	 * to the closest obstacle in the physical and virtual environments in front
	 * of the user.
	 * This value is limited by the user's perceptual thresholds.
	 * @param sim_state The state of the simulation on the current frame.
	 * @param egocentric_user The user whom we are computing redirection gains for.
	 * @return The translation gain to apply.
	 */
	float set_translation_gain(simulation_state& sim_state, user* egocentric_user);

	/**
	 * Sets the curvature gain according to the distance between the distances
	 * to the closest obstacle in the physical and virtual environments to the
	 * left and right of the user.
	 * This value is limited by the user's perceptual thresholds.
	 * @param direction The direction to steer the user with the curvature gain. Positive value is to the left, negative is to the right.
	 */
	float set_curvature_gain(int direction);

	const int SAMPLE_RATE = 20; // Number of distances to check when resetting.
	const float REDIRECTION_LOSS_THRESHOLD = 0.1f; // Alignment threshold for when to turn off redirection.
	const float ROTA_GAIN_SMOOTHING = 0.125f; // Smoothing parameter for interpoating rotation gains between frames.

	std::vector<float> cur_alignment;
	float prev_loss;
	float cur_loss;
	float cur_north_loss;
	float cur_east_loss;
	float cur_west_loss;
	float prev_rota_gain;
};