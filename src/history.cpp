#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <time.h>
#include <iomanip>
#include <chrono>
#include <filesystem>

#ifdef _WIN32
#include <direct.h>
#endif
#ifdef linux
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "history.h"
#include "timestep.h"
#include "user.h"
#include "config.h"

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

	#ifdef _WIN32
	struct stat info;
	if (stat(config::DATA_DIR, &info) != 0) {
		printf("cannot access %s, so the directory was created.\n", config::DATA_DIR);
	}
	if (!(info.st_mode & S_IFDIR)) {  // S_ISDIR() doesn't exist on my windows 
		_mkdir(config::DATA_DIR);
	}
	#endif
	#ifdef linux
	struct stat st = { 0 };

	if (stat(config::DATA_DIR, &st) == -1) {
		mkdir(config::DATA_DIR, 0700);
	}
	#endif
	
	time_t t = std::time(0);
	struct tm* now = localtime(&t);

	char formatted_time[80];
	char temp[50];
	char filename[300];
	std::strcpy(filename, config::DATA_DIR);
	sprintf(temp, "/user_%d_movement_history_path_#%d_", user_to_write->id, path_number);
	std::strcat(filename, temp);
	strftime(formatted_time, 80, "%Y-%m-%d-%H-%M-%S.csv", now);
	std::strcat(filename, formatted_time);

	std::ofstream my_file;
	my_file.open(filename);
	if (my_file.is_open()) {
		my_file << "timestamp,phys_pos_x,phys_pos_y,virt_pos_x,virt_pos_y,phys_heading,virt_heading,apply_rota,rota_gain,apply_trans,trans_gain,apply_curve,curve_gain,curve_dir,nav_state,collision\n";
		for (int i = 0; i < time.size(); i++) {
			redirection_unit redir = rdw_history[i];

			my_file << std::fixed << std::setprecision(10) << time[i] << ","; // timestamp
			my_file << phys_pos_history[i].x << "," << phys_pos_history[i].y << ","; // phys_pos_x, phys_pos_y
			my_file << virt_pos_history[i].x << "," << virt_pos_history[i].y << ","; // virt_pos_x, virt_pos_y
			my_file << std::fixed << std::setprecision(10) << phys_heading_history[i] << ","; // phys_heading
			my_file << std::fixed << std::setprecision(10) << virt_heading_history[i] << ","; // virt_heading

			// apply_rota, rota_gain
			if (redir.apply_rota) {
				my_file << "true," << std::fixed << std::setprecision(10) << redir.rota_gain << ",";
			}
			else {
				my_file << "false," << std::fixed << std::setprecision(10) << redir.rota_gain << ",";
			}
			// apply_trans, trans_gain
			if (redir.apply_trans) {
				my_file << "true," << std::fixed << std::setprecision(10) << redir.trans_gain << ",";
			}
			else {
				my_file << "false," << std::fixed << std::setprecision(10) << redir.trans_gain << ",";
			}
			// apply_curve, curve_gain, curve_dir
			if (redir.apply_curve) {
				my_file << "true," << std::fixed << std::setprecision(10) << redir.curve_gain << "," << redir.curve_gain_dir << ",";
			}
			else {
				my_file << "false," << std::fixed << std::setprecision(10) << redir.curve_gain << "," << redir.curve_gain_dir << ",";
			}

			// nav_state
			switch (nav_state_history[i]) {
			case user_state::NAVIGATION_STATE::OK:
				my_file << "OK" << ",";
				break;
			case user_state::NAVIGATION_STATE::RESETTING:
				my_file << "RESETTING" << ",";
				break;
			}

			// collision
			my_file << collision_history[i];

			my_file << "\n";
		}
	}
	my_file.close();

	char buffer[300];
	temp[200];
	std::strcpy(buffer, config::DATA_DIR);
	sprintf(temp, "/user_%d_config.txt", user_to_write->id);
	std::strcat(buffer, temp);
	my_file.open(buffer);
	if (my_file.is_open()) {
		my_file << "Radius: " << user_to_write->radius << "\n";
		my_file << "Number of collisions: " << user_to_write->num_collisions << "\n";
		my_file << "Angular velocity: " << user_to_write->state.get_angular_vel() << "\n";
		my_file << "Walking velocity: " << user_to_write->state.get_velocity() << "\n";
		my_file << "Redirector: " << user_to_write->get_redirector_name() << "\n";
		my_file << "Resetter: " << user_to_write->get_resetter_name() << "\n";
		my_file << "Min waypoint distance: " << config::MIN_WAYPOINT_DISTANCE << "\n";
		my_file << "Max waypoint distance: " << config::MAX_WAYPOINT_DISTANCE << "\n";
		my_file << "Min waypoint angle: " << config::MIN_WAYPOINT_ANGLE << "\n";
		my_file << "Max waypoint angle: " << config::MAX_WAYPOINT_ANGLE << "\n";
		my_file << "Path model: " << user_to_write->state.model.get_path_model_name() << "\n";
	}
	my_file.close();
}