#include "timestep.h"

namespace timestep {
	float elapsed_time = 0.0f;
	int num_timesteps = 0;
	time_t start_time = time(NULL);

	void add_time() {
		elapsed_time += dt;
		num_timesteps++;
	}
}