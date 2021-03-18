#pragma once

#include <vector>

#include "user_state.hpp"
#include "vec2f.hpp"
#include "redirector.h"

class user;

class history {
	public:
		history();
		void log(user_state state);
		void write(user* user_to_write, int path_number);
		void reset();

	private:
		std::vector<float> time;
		std::vector<vec2f> phys_pos_history;
		std::vector<vec2f> virt_pos_history;
		std::vector<float> phys_heading_history;
		std::vector<float> virt_heading_history;
		std::vector<redirection_unit> rdw_history;
		std::vector<float> north_alignment_history;
		std::vector<float> east_alignment_history;
		std::vector<float> west_alignment_history;
		std::vector<user_state::NAVIGATION_STATE> nav_state_history;
		std::vector<bool> collision_history;
		int num_collisions;
		float time_in_reset;
};