#pragma once

#include <vector>

#include "user_state.h"
#include "vec2f.h"
#include "redirector.h"

class user;

class history {
	public:
		/**
		 * Default constructor for the history logger class.
		 */
		history();

		/**
		 * Record all the simulation data on the current timestep.
		 * @param state The state of the user.
		 */
		void log(user_state state);

		/**
		 * Write the logged data to files.
		 * @param user_to_write The user whose data is being written to file.
		 * @param path_number The number of the path which is being written.
		 */
		void write(user* user_to_write, int path_number);

		/**
		 * Clear all the logged history data.
		 */
		void reset();

	private:
		std::vector<float> time;
		std::vector<vec2f> phys_pos_history;
		std::vector<vec2f> virt_pos_history;
		std::vector<float> phys_heading_history;
		std::vector<float> virt_heading_history;
		std::vector<redirection_unit> rdw_history;
		std::vector<user_state::NAVIGATION_STATE> nav_state_history;
		std::vector<bool> collision_history;
		int num_collisions;
		float time_in_reset;
};