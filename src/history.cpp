#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <time.h>
#include <iomanip>
#include <chrono>

#include "history.h"
#include "timestep.h"
#include "user.h"
#include "config.h"
#include "ext/ghc/fs_std.hpp"

history::history() {
	num_collisions = 0;
	time_in_reset = 0.0f;
}

void history::log(user_state state) {
	time.push_back(timestep::elapsed_time);
	phys_pos_history.push_back(state.get_phys_pos());
	virt_pos_history.push_back(state.get_virt_pos());
	phys_heading_history.push_back(state.get_phys_heading());
	virt_heading_history.push_back(state.get_virt_heading());
	rdw_history.push_back(state.get_prev_redir());

	if (nav_state_history.size() == 0) {
		collision_history.push_back(0);
	}
	else {
		if (nav_state_history.back() == user_state::NAVIGATION_STATE::OK &&
			state.nav_state == user_state::NAVIGATION_STATE::RESETTING) {
			collision_history.push_back(1);
		}
		else {
			collision_history.push_back(0);
		}
	}

	nav_state_history.push_back(state.nav_state);

	if (state.nav_state == user_state::NAVIGATION_STATE::RESETTING) 
		time_in_reset += timestep::dt;
}

void history::reset() {
	time.clear();
	phys_pos_history.clear();
	virt_pos_history.clear();
	phys_heading_history.clear();
	virt_heading_history.clear();
	rdw_history.clear();
	nav_state_history.clear();
	collision_history.clear();
	time_in_reset = 0.0f;
}

void history::write(user* user_to_write, int path_number) {
	std::cout.precision(10);

	// Create output directory if it doesn't exist
	if (!fs::exists(config::DATA_DIR)) {
		fs::create_directory(config::DATA_DIR);
	}

	// Log the users' position data across all paths walked
	time_t t = std::time(0);
	struct tm* now = localtime(&t);
	char user_and_path[50];
	sprintf(user_and_path, "user_%d_movement_history_path_#%d_", user_to_write->id, path_number);
	char formatted_time[80];
	strftime(formatted_time, 80, "%Y-%m-%d-%H-%M-%S.csv", now);
	fs::path filename; // empty path
	filename += user_and_path;
	filename += formatted_time;
	fs::path log_file = config::DATA_DIR / fs::path(filename);

	std::ofstream ofs;
	ofs.open(log_file);
	if (ofs.is_open()) {
		ofs << "timestamp,phys_pos_x,phys_pos_y,virt_pos_x,virt_pos_y,phys_heading,virt_heading,apply_rota,rota_gain,apply_trans,trans_gain,apply_curve,curve_gain,curve_dir,nav_state,collision\n";
		for (int i = 0; i < time.size(); i++) {
			redirection_unit redir = rdw_history[i];

			ofs << std::fixed << std::setprecision(10) << time[i] << ","; // timestamp
			ofs << phys_pos_history[i].x << "," << phys_pos_history[i].y << ","; // phys_pos_x, phys_pos_y
			ofs << virt_pos_history[i].x << "," << virt_pos_history[i].y << ","; // virt_pos_x, virt_pos_y
			ofs << std::fixed << std::setprecision(10) << phys_heading_history[i] << ","; // phys_heading
			ofs << std::fixed << std::setprecision(10) << virt_heading_history[i] << ","; // virt_heading

			// apply_rota, rota_gain
			if (redir.apply_rota) {
				ofs << "true," << std::fixed << std::setprecision(10) << redir.rota_gain << ",";
			}
			else {
				ofs << "false," << std::fixed << std::setprecision(10) << redir.rota_gain << ",";
			}
			// apply_trans, trans_gain
			if (redir.apply_trans) {
				ofs << "true," << std::fixed << std::setprecision(10) << redir.trans_gain << ",";
			}
			else {
				ofs << "false," << std::fixed << std::setprecision(10) << redir.trans_gain << ",";
			}
			// apply_curve, curve_gain, curve_dir
			if (redir.apply_curve) {
				ofs << "true," << std::fixed << std::setprecision(10) << redir.curve_gain << "," << redir.curve_gain_dir << ",";
			}
			else {
				ofs << "false," << std::fixed << std::setprecision(10) << redir.curve_gain << "," << redir.curve_gain_dir << ",";
			}

			// nav_state
			switch (nav_state_history[i]) {
			case user_state::NAVIGATION_STATE::OK:
				ofs << "OK" << ",";
				break;
			case user_state::NAVIGATION_STATE::RESETTING:
				ofs << "RESETTING" << ",";
				break;
			}

			// collision
			ofs << collision_history[i];

			ofs << "\n";
		}
	}
	ofs.close();

	// Log the user configuration data (things like user velocity and 
	// motion models).
	char temp[200];
	sprintf(temp, "user_%d_config.txt", user_to_write->id);
	fs::path config_file = config::DATA_DIR / fs::path(temp);
	ofs.open(config_file);
	if (ofs.is_open()) {
		ofs << "Radius: " << user_to_write->radius << "\n";
		ofs << "Number of collisions: " << user_to_write->num_collisions << "\n";
		ofs << "Angular velocity: " << user_to_write->state.get_angular_vel() << "\n";
		ofs << "Walking velocity: " << user_to_write->state.get_velocity() << "\n";
		ofs << "Redirector: " << user_to_write->get_redirector_name() << "\n";
		ofs << "Resetter: " << user_to_write->get_resetter_name() << "\n";
		ofs << "Min waypoint distance: " << config::MIN_WAYPOINT_DISTANCE << "\n";
		ofs << "Max waypoint distance: " << config::MAX_WAYPOINT_DISTANCE << "\n";
		ofs << "Min waypoint angle: " << config::MIN_WAYPOINT_ANGLE << "\n";
		ofs << "Max waypoint angle: " << config::MAX_WAYPOINT_ANGLE << "\n";
		ofs << "Path model: " << user_to_write->state.model.get_path_model_name() << "\n";
	}
	ofs.close();
}