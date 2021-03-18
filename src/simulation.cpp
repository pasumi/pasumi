#include <iostream>
#include <fstream>
#include <chrono>

// #include "pybind11/pybind11.h"
// #include <pybind11/pybind11.h>
// #include <vector>
// #include "user.hpp"
#include "simulation.hpp"
#include "timestep.h"
#include "config.h"

simulation::simulation() {

}

float simulation::get_environment_clutter(physical_environment* env) {
	std::vector<float> clutter_vals;
	float step_size = 0.5f;
	float theta_step_size = math::radians(1);
	float f = env->min_x + step_size;

	for (float x = env->min_x + step_size; x < env->max_x; x += step_size) {
		for (float y = env->min_y + step_size; y < env->max_y; y += step_size) {
			vec2f p = vec2f(x, y);
			if (!env->point_is_legal(p)) continue;
			p = vec2f(math::normalize_in_range(x, env->min_x, env->max_x),
					  math::normalize_in_range(y, env->min_y, env->max_y));
			float clutter = 0.0f;

			for (float theta = 0.0f; theta < math::two_pi; theta += theta_step_size) {
				vec2f dir = rad_2_vec(theta);
				float closest = math::max_float;

				for (wall* w : env->get_walls()) {
					vec2f* s1 = w->get_vertices()[0];
					s1 = new vec2f(math::normalize_in_range(s1->x, env->min_x, env->max_x),
							   math::normalize_in_range(s1->y, env->min_y, env->max_y));
					vec2f* s2 = w->get_vertices()[1];
					s2 = new vec2f(math::normalize_in_range(s2->x, env->min_x, env->max_x),
							   math::normalize_in_range(s2->y, env->min_y, env->max_y));
					float t = geom::ray_line_intersect(&p, &dir, s1, s2);
					if (t >= 0.0f && t <= closest) {
						closest = t;
					}
				}
				for (obstacle* o : env->get_obstacles()) {
					for (wall* w : o->get_walls()) {
						vec2f* s1 = w->get_vertices()[0];
						s1 = new vec2f(math::normalize_in_range(s1->x, env->min_x, env->max_x),
							math::normalize_in_range(s1->y, env->min_y, env->max_y));
						vec2f* s2 = w->get_vertices()[1];
						s2 = new vec2f(math::normalize_in_range(s2->x, env->min_x, env->max_x),
							math::normalize_in_range(s2->y, env->min_y, env->max_y));
						float t = geom::ray_line_intersect(&p, &dir, s1, s2);
						if (t >= 0.0f && t <= closest) {
							closest = t;
						}
					}
				}

				clutter += closest;
			}

			clutter_vals.push_back(clutter / 360.0f);
		}
	}

	float clutter_sum = 0.0f;
	for (float c : clutter_vals) { clutter_sum += c; }
	clutter_sum /= clutter_vals.size();
	return clutter_sum;
}

float simulation::get_environment_clutter(virtual_environment* env) {
	std::vector<float> clutter_vals;
	float step_size = 0.5f;
	float theta_step_size = math::radians(1);
	float f = env->min_x + step_size;

	for (float x = env->min_x + step_size; x < env->max_x; x += step_size) {
		for (float y = env->min_y + step_size; y < env->max_y; y += step_size) {
			vec2f p = vec2f(x, y);
			if (!env->point_is_legal(p)) continue;
			p = vec2f(math::normalize_in_range(x, env->min_x, env->max_x),
				math::normalize_in_range(y, env->min_y, env->max_y));
			float clutter = 0.0f;

			for (float theta = 0.0f; theta < math::two_pi; theta += theta_step_size) {
				vec2f dir = rad_2_vec(theta);
				float closest = math::max_float;

				for (wall* w : env->get_walls()) {
					vec2f* s1 = w->get_vertices()[0];
					s1 = new vec2f(math::normalize_in_range(s1->x, env->min_x, env->max_x),
						math::normalize_in_range(s1->y, env->min_y, env->max_y));
					vec2f* s2 = w->get_vertices()[1];
					s2 = new vec2f(math::normalize_in_range(s2->x, env->min_x, env->max_x),
						math::normalize_in_range(s2->y, env->min_y, env->max_y));
					float t = geom::ray_line_intersect(&p, &dir, s1, s2);
					if (t >= 0.0f && t <= closest) {
						closest = t;
					}
				}
				for (obstacle* o : env->get_obstacles()) {
					for (wall* w : o->get_walls()) {
						vec2f* s1 = w->get_vertices()[0];
						s1 = new vec2f(math::normalize_in_range(s1->x, env->min_x, env->max_x),
							math::normalize_in_range(s1->y, env->min_y, env->max_y));
						vec2f* s2 = w->get_vertices()[1];
						s2 = new vec2f(math::normalize_in_range(s2->x, env->min_x, env->max_x),
							math::normalize_in_range(s2->y, env->min_y, env->max_y));
						float t = geom::ray_line_intersect(&p, &dir, s1, s2);
						if (t >= 0.0f && t <= closest) {
							closest = t;
						}
					}
				}

				clutter += closest;
			}

			clutter_vals.push_back(clutter / 360.0f);
		}
	}

	float clutter_sum = 0.0f;
	for (float c : clutter_vals) { clutter_sum += c; }
	clutter_sum /= clutter_vals.size();
	return clutter_sum;
}

void simulation::compute_clutter(physical_environment* p_env, virtual_environment* v_env) {
	float phys_clutter = get_environment_clutter(p_env);
	std::cout << "Physical room clutter: " << phys_clutter << std::endl;
	float virt_clutter = get_environment_clutter(v_env);
	std::cout << "Virtual room clutter: " << virt_clutter << std::endl;

	std::cout << "Physical/Virtual clutter ratio: " << phys_clutter / virt_clutter << std::endl;
	system("pause");
}

simulation::simulation(std::vector<physical_environment*> physical_envs, virtual_environment* virtual_env, std::vector<user*> users){
    this->phys_envs = physical_envs;
    this->virt_env = virtual_env;

	//compute_clutter();

    this->users = users;
    this->done = false;

    for (user* user : users) {
		//user->add_phys_env(phys_env);
        //user->add_virt_env(virt_env);
		user->add_other_users(users);
        user->generate_motion(virt_env);
    }

	check_simulation_parameters();
	sim_state = simulation_state(users, phys_envs, virt_env);
}

void simulation::check_simulation_parameters() {
	//assert(config::USER_RADIUS > 0.0f); // TODO: Allow this or not??? I think I can handle it numerically, but I don't know if it should be allowed.
}

void simulation::add_user(user* user_to_add) {
	// TODO: remove? or refactor to make useful? It isn't used anymore since i pass in a vector of user objects.
    users.push_back(user_to_add);
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


// // Python interface
// namespace py = pybind11;

// PYBIND11_MODULE(simulation, m) {
//     py::class_<Simulation>(m, "Simulation")
//         .def(py::init<>())
//         .def("add_user", &Simulation::add_user)
//         .def("step", &Simulation::step)
//         .def("print_status", &Simulation::print_status);
// }