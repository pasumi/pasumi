#pragma once

class user;

#include "simulation_state.h"
#include "vec2f.h"
#include "math.hpp"
#include "motion_model.h"
#include "timestep.h"
#include "proximity_container.h"

class resetter {
	public:
		resetter();
		~resetter();

		virtual float reset(simulation_state& sim_state, user* egocentric_user) = 0;
		char* name;
		int reset_timer;

	protected:
		float reorient_to_target(vec2f phys_heading, float virt_heading, vec2f phys_pos, vec2f virt_pos, vec2f target, std::vector<trajectory_unit>* path);
};