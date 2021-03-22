#include <iostream>
#include <limits>
#include <time.h>
#include <signal.h>

#include "virtual_environment.hpp"
#include "physical_environment.hpp"
#include "simulation.hpp"
#include "user.hpp"
#include "motion_model.h"
#include "timestep.h"
#include "apf_grad.h"
#include "config.h"
#include "arc.h"
#include "node.h"
//#include "vec2f.hpp"
//#include "geometry.hpp"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;
int EXIT_EARLY = 0;

void signal_callback_handler(int signum) {
	std::cout << "Caught signal " << signum << std::endl;
	EXIT_EARLY = 1;
}

int run_no_graphics() {
	// Register signal and signal handler
	signal(SIGINT, signal_callback_handler);

	simulation sim = simulation(config::phys_envs, &config::virt_env, config::users);

	time_t start_time;
	start_time = time(NULL);
	int frame_count = 0;

	while (!sim.done || EXIT_EARLY) {
		sim.step();
		frame_count++;

		if (EXIT_EARLY) {
			sim.quit_early();
			break;
		}
	}
	time_t end_time;
	end_time = time(NULL);
	sim.write(frame_count, start_time, end_time);

	std::cout << "Num frames: " << frame_count << std::endl;
	std::cout << "Elapsed real time: " << end_time - start_time << std::endl;
	std::cout << "FPS: " << frame_count / (end_time - start_time) << std::endl;

	return 0;
}

void experiment() {
	run_no_graphics();
}

int main(){
	experiment();

	return 0;
}