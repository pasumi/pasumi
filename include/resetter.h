#pragma once

class user;

#include "simulation_state.h"
#include "vec2f.h"
#include "math.hpp"
#include "motion_model.h"
#include "timestep.h"
#include "proximity_container.h"

/**
 * Base class for a resetter object. The resetter is responsible for determining which
 * direction the user will face after getting too close to a physical obstacle.
 */
class resetter {
	public:
		/**
		 * Default constructor. This is useless.
		 */
		resetter();

		~resetter();

		/**
		 * Implements the heuristic that determines which direction the user should 
		 * face after getting too close to an obstacle in the PE.
		 * @param sim_state The state of the simulation on the current frame.
		 * @param egocentric_user The user who is being reoriented.
		 * @return The direction that the user will face after reorientation.
		 */
		virtual float reset(simulation_state& sim_state, user* egocentric_user) = 0;

		char* name;
		int reset_timer; // Number of frames that it will take to complete the reorientation.

	protected:
		/**
		 * Inserts rotation movements into the user's path to reorient them in the
		 * physical environment such that they face in the direction determined by
		 * the reset() method. In reality, the user has to turn 360 degrees in the
		 * virtual world, and a redirection gain is applied such that their 
		 * corresponding physical rotation is not 360 degrees. By the end of the 360
		 * virtual turn, the user's physical heading will differ from their virtual
		 * heading, and they can resume walking. In the simulation, we don't actually 
		 * rotate the user around in the virtual environment since it's pointless and 
		 * because it could just introduce instability to the simulation due to 
		 * floating point errors.
		 * @param phys_heading The user's heading in the PE before reorientation.
		 * @param virt_heading The user's heading in the VE before reorientation.
		 * @param phys_pos The user's position in the PE.
		 * @param virt_pos The user's position in the VmE.
		 * @param target The direction that the user should face after reorientation.
		 * @param path The vector describing the user's trajectory.
		 * @return The rotation gain that should be applied to have the user turn the correct amount in the PE.
		 */
		float reorient_to_target(vec2f phys_heading, float virt_heading, vec2f phys_pos, vec2f virt_pos, vec2f target, std::vector<trajectory_unit>* path);
};