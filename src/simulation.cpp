#include <iostream>
#include <fstream>
#include <chrono>
#include <string.h>

#include "simulation.h"
#include "timestep.h"
#include "config.h"

simulation::simulation() {

}

simulation::simulation(std::vector<physical_environment*> physical_envs, virtual_environment* virtual_env, std::vector<user*> users){
    this->phys_envs = physical_envs;
    this->virt_env = virtual_env;


    this->users = users;
    this->done = false;

    for (user* user : users) {
		user->add_other_users(users);
        user->generate_motion(virt_env);
    }

	check_simulation_parameters();
	sim_state = simulation_state(users, phys_envs, virt_env);
}

void simulation::check_simulation_parameters() {
	assert(config::USER_RADIUS > 0.0f);
}

void simulation::step() {
    bool all_users_done = true;
	bool all_users_finished_path = true;

	for (physical_environment* p : phys_envs) {
		p->step();
	}
	virt_env->step();

	// Step each user
    for (user* user : users) { 
        if (!user->finished_path) {
            user->step(sim_state);
            all_users_done = false;
			all_users_finished_path = false;
        }
    }

	// Prepare users for their next path (reset position and generate new path)
	if (all_users_finished_path) {
		for (physical_environment* p : phys_envs) {
			for (obstacle* o : p->get_obstacles()) {
				if (o->is_dynamic()) {
					o->reset_state();
				}
			}
		}
		for (obstacle* o : virt_env->get_obstacles()) {
			if (o->is_dynamic()) {
				o->reset_state();
			}
		}
		for (user* user : users) {
			user->prep_for_next_path(sim_state);
			if (!user->done) all_users_done = false;
		}
	}

    done = all_users_done;
	timestep::add_time();
}

void simulation::write(int frame_count, time_t start_time, time_t end_time) {
	char filename[300];
	strcpy(filename, config::DATA_DIR);
	strcat(filename, "/config.txt");

	std::ofstream my_file;
	my_file.open(filename);
	if (my_file.is_open()) {
		my_file << "===PHYSICAL ENVIRONMENT===\n";
		my_file << "Center: " << config::phys_env1.get_center() << "\n";
		my_file << "---WALLS---\n";
		for (wall* wall : config::phys_env1.get_walls()) {
			vec2f* p = wall->get_vertices()[0];
			my_file << "\t(" << p->x << ", " << p->y << ")\n";
		}
		my_file << "---OBSTACLES---\n";
		int obs_count = 1;
		for (obstacle* obs : config::phys_env1.get_obstacles()) {
			my_file << "\t" << "o" << obs_count << "\n";
			for (vec2f* v : obs->get_vertices()) {
				my_file << "\t" << *v << "\n";
			}
			obs_count++;
		}

		my_file << "\n===VIRTUAL ENVIRONMENT===\n";
		my_file << "---WALLS---\n";
		for (wall* w : config::virt_env.get_walls()) {
			my_file << "\t" << *(w->get_vertices()[0]) << "\n";
		}
		my_file << "---OBSTACLES---\n";
		obs_count = 1;
		for (obstacle* obs : config::virt_env.get_obstacles()) {
			my_file << "\t" << "o" << obs_count << "\n";
			for (vec2f* v : obs->get_vertices()) {
				my_file << "\t" << *v << "\n";
			}
			obs_count++;
		}

		time_t end_time;
		end_time = time(NULL);
		my_file << "\n===SIMULATION STATS===\n";
		my_file << "Total frames run: " << frame_count << "\n";
		my_file << "Total elapsed real time: " << (end_time - start_time) << "\n";
		my_file << "Runtime FPS: " << frame_count / (end_time - start_time) << "\n";
		my_file << "Timestep size: " << timestep::dt << "\n";
		my_file << "Graphics on: " << config::GRAPHICS << "\n";
		my_file << "Number of users: " << config::users.size()<< "\n";
		my_file << "Reset distance threshold: " << config::DISTANCE_THRESHOLD << "\n";
		my_file << "Reset distance check value: " << config::RESET_DISTANCE_CHECK_VALUE << "\n";
		my_file << "Seeded: " << config::SEEDED << "\n";
		my_file << "Seed used: " << config::SEED << "\n";
		my_file << "Random physical starting positions: " << config::RANDOM_PHYS_START_POS << "\n";
		my_file << "Radius physical starting positions: " << config::RADIUS_PHYS_START_POS << "\n";
		my_file << "Radius physical starting threshold: " << config::RADIUS_PHYS_START_POS_THRESHOLD << "\n";
		my_file << "Random virtual starting positions: " << config::RANDOM_VIRT_START_POS << "\n";
		my_file << "Match physical and virtual positions: " << config::MATCH_PHYS_VIRT_POS << "\n";
		my_file << "Random physical heading: " << config::RANDOM_PHYS_HEADING << "\n";
		my_file << "Random virtual heading: " << config::RANDOM_VIRT_HEADING << "\n";
		my_file << "Match physical and virtual headings: " << config::MATCH_PHYS_VIRT_HEADING << "\n";
		my_file << "Number of paths: " << config::NUM_PATHS << "\n";
		my_file << "Number of waypoints: " << config::NUM_WAYPOINTS << "\n";
	}
	my_file.close();
}

void simulation::render() {

}

void simulation::quit_early() {
	for (user* u : users) {
		u->write(1);
	}
}
