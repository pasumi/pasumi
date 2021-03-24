#include <cstdlib>
#include <ctime>
#include <time.h>
#include <iostream>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>

#include "motion_model.h"
#include "math.hpp"
#include "config.h"

motion_model::motion_model() {

}

motion_model::motion_model(int waypoints, int paths, PATH_MODEL path_model) {
	this->waypoints = waypoints;
	this->paths = paths;
	this->path_model = path_model;

	PATH_MODEL_STRINGS = { "RANDOM", "STRAIGHT", "FILE" };
}

char* motion_model::get_path_model_name() {
	return PATH_MODEL_STRINGS[(int)path_model];
}

void motion_model::generate_path(std::vector<trajectory_unit>& user_path, virtual_environment* virt_env, vec2f cur_virt_pos, float cur_virt_heading) {
	switch (path_model) {
	case PATH_MODEL::RANDOM:
	{
		float angle_per_dt = math::radians(config::ANGULAR_VELOCITY) / (1.0f / timestep::dt);
		float meters_per_dt = config::VELOCITY / (1.0f / timestep::dt);

		vec2f cur_pos = cur_virt_pos;
		float cur_heading = cur_virt_heading;

		path next_path = path();
		for (int j = 0; j < waypoints; j++) {
			vec2f sampled_point = sample_point(virt_env, cur_pos);
			generate_path_to_point(cur_pos, cur_heading, sampled_point, angle_per_dt, meters_per_dt, user_path);
			cur_pos = sampled_point;
		}
		cur_path = next_path;
		all_paths.push_back(next_path);
	}
	break;
	case PATH_MODEL::STRAIGHT:
	{
		float angle_per_dt = math::radians(config::ANGULAR_VELOCITY) / (1.0f / timestep::dt);
		float meters_per_dt = config::VELOCITY / (1.0f / timestep::dt);
		vec2f cur_pos = cur_virt_pos;
		float cur_heading = cur_virt_heading;
		path next_path = path();
		vec2f dir = rad_2_vec(cur_virt_heading);
		vec2f goal = cur_pos;
		goal += (normalize(dir) * 4.0f);
		generate_path_to_point(cur_pos, cur_heading, goal, angle_per_dt, meters_per_dt, user_path);
		cur_path = next_path;
		all_paths.push_back(next_path);
	}
	break;
	case PATH_MODEL::FILE:
	{
		std::fstream newfile;
		newfile.open(config::path_file, std::ios::in);
		path new_path = path();
		std::vector<vec2f> pts;

		if (newfile.is_open()) {   
			std::string tp;
			while (getline(newfile, tp)) { 
				std::stringstream ss(tp);
				int i = 0;
				vec2f next_point;
				while (ss.good()) {
					std::string substr;
					getline(ss, substr, ',');
					if (i % 2 == 0) next_point.x = std::stof(substr);
					else			next_point.y = std::stof(substr);
					i++;
				}
				pts.push_back(next_point);
			}
			newfile.close(); 
		}

		float angle_per_dt = math::radians(config::ANGULAR_VELOCITY) / (1.0f / timestep::dt);
		float meters_per_dt = config::VELOCITY / (1.0f / timestep::dt);

		vec2f cur_pos = cur_virt_pos;
		float cur_heading = cur_virt_heading;
		for (vec2f p : pts) {
			generate_path_to_point(cur_pos, cur_heading, p, angle_per_dt, meters_per_dt, user_path);
			cur_pos = p;
		}

		cur_path = new_path;
		all_paths.push_back(new_path);
	}
	break;
	}
}

void motion_model::generate_path_to_point(vec2f cur_pos, float& cur_heading, vec2f goal_point, float angle_per_dt, float meters_per_dt, std::vector<trajectory_unit>& user_path) {
	vec2f next_dir = normalize(goal_point - cur_pos);
	float angle_change = signed_angle(rad_2_vec(cur_heading), next_dir);
	float pos_distance = length(cur_pos - goal_point);

	int num_iter = math::abs(int(angle_change / angle_per_dt));
	for (int i = 0; i < num_iter; i++) {
		cur_heading += angle_per_dt * math::sign(angle_change);
		user_path.push_back(trajectory_unit(cur_pos.x, cur_pos.y, cur_heading));
	}
	float remainder = math::abs(angle_change) - (angle_per_dt * num_iter);
	if (remainder) {
		cur_heading += remainder * math::sign(angle_change);
		user_path.push_back(trajectory_unit(cur_pos.x, cur_pos.y, cur_heading));
	}

	num_iter = int(pos_distance / meters_per_dt);
	for (int i = 1; i < num_iter+1; i++) {
		float x_offset = math::cos(cur_heading) * meters_per_dt * i;
		float y_offset = math::sin(cur_heading) * meters_per_dt * i;
		vec2f next_pos = cur_pos + vec2f(x_offset, y_offset);
		user_path.push_back(trajectory_unit(next_pos.x, next_pos.y, cur_heading));
	}
	remainder = pos_distance - (meters_per_dt * num_iter);
	if (remainder) {
		user_path.push_back(trajectory_unit(goal_point.x, goal_point.y, cur_heading));
	}
	else {
		user_path[user_path.size() - 1] = trajectory_unit(goal_point.x, goal_point.y, cur_heading);
	}
}

vec2f motion_model::sample_point(virtual_environment* virt_env, vec2f start_pos) {
	assert(virt_env->point_is_legal(start_pos));

	float distance_offset = config::MIN_WAYPOINT_DISTANCE + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (config::MAX_WAYPOINT_DISTANCE - config::MIN_WAYPOINT_DISTANCE)));
	float angle_offset = config::MIN_WAYPOINT_ANGLE + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (config::MAX_WAYPOINT_ANGLE - config::MIN_WAYPOINT_ANGLE)));

	vec2f sampled_point = (rad_2_vec(angle_offset) * distance_offset) + start_pos;
	bool path_is_legal = check_path_for_collision((environment*)virt_env, start_pos, sampled_point);
	bool is_far_enough_away = virt_env->get_closest_obstacle_distance(sampled_point) > config::RESET_DISTANCE_CHECK_VALUE;
	while (!virt_env->point_is_legal(sampled_point) || !path_is_legal || !is_far_enough_away) {
		distance_offset = config::MIN_WAYPOINT_DISTANCE + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (config::MAX_WAYPOINT_DISTANCE - config::MIN_WAYPOINT_DISTANCE)));
		angle_offset = config::MIN_WAYPOINT_ANGLE + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (config::MAX_WAYPOINT_ANGLE - config::MIN_WAYPOINT_ANGLE)));
		sampled_point = (rad_2_vec(angle_offset) * distance_offset) + start_pos;

		path_is_legal = check_path_for_collision((environment*)virt_env, start_pos, sampled_point);
		is_far_enough_away = virt_env->get_closest_obstacle_distance(sampled_point) > config::RESET_DISTANCE_CHECK_VALUE;
	}

	assert(virt_env->point_is_legal(sampled_point) && "Sampled virtual path point is not legal.");
	return sampled_point;
}

bool motion_model::check_path_for_collision(environment* env, vec2f p1, vec2f p2) {
	vec2f offset = normalize(p1 - p2);
	vec2f up = vec2f(-offset.y, offset.x) * config::USER_RADIUS;
	vec2f down = vec2f(offset.y, -offset.x) * config::USER_RADIUS;
	vec2f s1_p1 = p1 + up;
	vec2f s1_p2 = p2 + up;
	vec2f s2_p1 = p1 + down;
	vec2f s2_p2 = p2 + down;

	for (wall* w : env->get_walls()) {
		vec2f* w_p1 = w->get_vertices()[0];
		vec2f* w_p2 = w->get_vertices()[1];
		if (geom::line_line_intersect(&s1_p1, &s1_p2, w_p1, w_p2) ||
			geom::line_line_intersect(&s2_p1, &s2_p2, w_p1, w_p2) ||
			geom::line_line_intersect(&p1, &p2, w_p1, w_p2)) {
			return false;
		}
	}
	for (obstacle* o : env->get_obstacles()) {
		for (wall* w : o->get_walls()) {
			vec2f* w_p1 = w->get_vertices()[0];
			vec2f* w_p2 = w->get_vertices()[1];
			if (geom::line_line_intersect(&s1_p1, &s1_p2, w_p1, w_p2) ||
				geom::line_line_intersect(&s2_p1, &s2_p2, w_p1, w_p2) ||
				geom::line_line_intersect(&p1, &p2, w_p1, w_p2)) {
				return false;
			}
		}
	}
	return true;
}
