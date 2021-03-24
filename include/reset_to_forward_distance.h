#pragma once

#include "resetter.h"
#include "geometry.h"
#include "user.h"

class reset_to_forward_distance : public resetter {
	public:
		reset_to_forward_distance();
		~reset_to_forward_distance();
		float reset(simulation_state& sim_state, user* egocentric_user);

	private:
		const int SAMPLE_RATE = 20;
		std::vector<float> sample_directions;
};