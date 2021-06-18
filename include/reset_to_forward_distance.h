#pragma once

#include "resetter.h"
#include "geometry.h"
#include "user.h"

/**
 * Resetter class that implements the reset heuristic introduced by Williams et al. in
 * "ARC: Alignment-based Redirection Controller for Redirected Walking in Complex 
 * Environments."
 */
class reset_to_forward_distance : public resetter {
	public:
		/**
		 * Default constructor.
		 */
		reset_to_forward_distance();

		~reset_to_forward_distance();

		/**
		 * Resets the user according to the heuristic developed by Williams et al.
		 * This heuristic reorients the user away from the nearby obstacle, in a 
		 * direction for which the distance to the closest obstacle in that direction 
		 * (in the PE) is most similar to the distance to the closest obstacle in 
		 * front of the user in the VE. It samples 20 distances around the user and 
		 * chooses the reset direction from those 20. In addition to requiring that 
		 * the user faces away from the nearby obstacle, the distance in the direction
		 * that they face in the PE must be greater than or equal to that distance in 
		 * the VE. If no such direction exists, the direction in the PE for which the 
		 * distance to the closest obstacle is closest to that distance in the VE is 
		 * chosen. You should refer to the paper for a diagram.
		 * @param sim_state The state of the simulation on the current frame.
		 * @param egocentric_user The user who is being redirected.
		 * @return The direcion that the user will face in the PE after resetting.
		 */
		float reset(simulation_state& sim_state, user* egocentric_user);

	private:
		const int SAMPLE_RATE = 20; // The number of directions aournd the user that are sampled to compute the distance to obstacles.
		std::vector<float> sample_directions;
};