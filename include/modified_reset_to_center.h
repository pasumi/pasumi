#pragma once

#include "resetter.h"

class modified_reset_to_center : public resetter {
	public:
		modified_reset_to_center();
		~modified_reset_to_center();
		float reset(simulation_state& sim_state, user* egocentric_user);

	private:
};
