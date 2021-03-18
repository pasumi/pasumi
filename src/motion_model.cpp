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

motion_model::motion_model(int waypoints, int paths, PATH_MODEL path_model, TRAJECTORY_MODEL trajectory_model) {
	this->waypoints = waypoints;
	this->paths = paths;
	this->path_model = path_model;
	this->trajectory_model = trajectory_model;

	if (path_model == PATH_MODEL::RVO) {
		//load_RVO_data();
	}

	PATH_MODEL_STRINGS = { "RANDOM", "STRAIGHT", "AZMANDIAN", "TEST", "FILE", "RVO" };
	TRAJECTORY_MODEL_STRINGS = { "STRAIGHT", "ROTATE", "SMOOTH" };
}

void motion_model::load_RVO_data() {
	char* path_dir = "C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/paths/";
	for (int i = 1; i <= 100; i++) {
		char filename[300];
		char temp[50];
		strcpy(filename, path_dir);
		sprintf(temp, "agent_0_path_#%d_movement_history.csv", i);
		strcat(filename, temp);

		std::fstream newfile;
		newfile.open(filename, std::ios::in);
		std::vector<vec2f> pts;
		
		bool first_line = true;
		if (newfile.is_open()) {
			std::string tp;
			while (getline(newfile, tp)) {
				if (first_line) {
					first_line = false;
					continue;
				}
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
				//new_path.waypoints.push_back(next_point);
				pts.push_back(next_point);
			}
			newfile.close();
		}

		std::vector<trajectory_unit> new_path;
		// Create the path from the points we just read in
		for (int j = 0; j < pts.size(); j++) {
			vec2f p1 = pts[j];
			vec2f p2 = pts[(j+1)%pts.size()];
			float heading = vec_2_rad(normalize(p2 - p1));
			int t = 4;
			if ((j + 1) % pts.size() != 0) {
				new_path.push_back(trajectory_unit(p1.x, p1.y, heading));
			}
			else{
				new_path.push_back(trajectory_unit(p1.x, p1.y, new_path.back().theta));
			}
		}
		rvo_paths.push_back(new_path);
	}
	std::cout << "Finished reading RVO data paths for user.\n";
}

char* motion_model::get_path_model_name() {
	return PATH_MODEL_STRINGS[(int)path_model];
}

char* motion_model::get_trajectory_model_name() {
	return TRAJECTORY_MODEL_STRINGS[(int)trajectory_model];
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
	case PATH_MODEL::AZMANDIAN:
	{
		config::MIN_WAYPOINT_DISTANCE = 2.0f;
		config::MAX_WAYPOINT_DISTANCE = 6.0f;
		config::MIN_WAYPOINT_ANGLE = -math::pi;
		config::MAX_WAYPOINT_ANGLE = math::pi;
		vec2f cur_pos = cur_virt_pos;
		path next_path = path();
		for (int j = 0; j < waypoints; j++) {
			vec2f sampled_point = sample_point(virt_env, cur_pos);
			//next_path.waypoints.push_back(sampled_point);
			cur_pos = sampled_point;
		}
		cur_path = next_path;
		all_paths.push_back(next_path);
	}
	break;
	case PATH_MODEL::TEST:
	{
		path next_path = path();
		trajectory_unit cur;
		//for (int i = 0; i < 40; i++) {
		float d = math::radians(1);
		for (float i = math::pi; i > math::pi / 2.0f; i -= d){
		//while (cur.x < 23.5619449f / 2.0f){
			cur.x = cos(i);
			cur.y = sin(i);
			cur.theta = i - (math::pi / 2.0f);
			next_path.waypoints.push_back(cur);
			user_path.push_back(cur);
		}
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
				//new_path.waypoints.push_back(next_point);
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
	case PATH_MODEL::RVO:
	{
		user_path.clear();
		get_RVO_path(user_path, virt_env, cur_virt_pos, cur_virt_heading);

		//for (auto p : rvo_paths.front()) {
			//user_path.push_back(p);
		//}
		//rvo_paths.erase(rvo_paths.begin());

	}
	break;
	}
}

//#include "Vector2.h"
#include "include/rvo/RVO.h"
#include "include/rvo/RVOSimulator.h"
#include "include/rvo/RVOSimulator.cpp"
#include "include/rvo/Definitions.h"
#include "include/rvo/Agent.h"
#include "include/rvo/Agent.cpp"
#include "include/rvo/KdTree.h"
#include "include/rvo/KdTree.cpp"
#include "include/rvo/Obstacle.h"
#include "include/rvo/Obstacle.cpp"
void motion_model::get_RVO_path(std::vector<trajectory_unit>& user_path, virtual_environment* virt_env, vec2f cur_virt_pos, float cur_virt_heading) {
	std::cout << "getting rvo paths" << std::endl;
	// Set up RVO simulator
	std::vector<std::vector<vec2f*>> new_obstacle_paths;
	std::vector<std::vector<RVO::Vector2>> goals;
	std::vector<std::vector<RVO::Vector2>> history;
	RVO::RVOSimulator* sim = new RVO::RVOSimulator();

	// Add obstacles to the simulation
	/*for (int i = 0; i < virt_env->get_walls().size(); i++) {
		auto w = virt_env->get_walls()[i];
		auto p1 = w->get_vertices()[0];
		auto p2 = w->get_vertices()[1];
		std::vector<RVO::Vector2> o;
		o.push_back(RVO::Vector2(p1->x, p1->y));
		o.push_back(RVO::Vector2(p2->x, p2->y));
		sim->addObstacle(o);
	}*/
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
	obstacle1.push_back(RVO::Vector2(-6.0f, 5.0f));
	obstacle1.push_back(RVO::Vector2(-6.0f, -5.0f));
	obstacle1.push_back(RVO::Vector2(-5.0f, -5.0f));
	obstacle1.push_back(RVO::Vector2(-5.0f, 5.0f));

	obstacle2.push_back(RVO::Vector2(-5.0f, -5.0f));
	obstacle2.push_back(RVO::Vector2(-5.0f, -6.0f));
	obstacle2.push_back(RVO::Vector2(5.0f, -6.0f));
	obstacle2.push_back(RVO::Vector2(5.0f, -5.0f));

	obstacle3.push_back(RVO::Vector2(5.0f, -5.0f));
	obstacle3.push_back(RVO::Vector2(6.0f, -5.0f));
	obstacle3.push_back(RVO::Vector2(6.0f, 5.0f));
	obstacle3.push_back(RVO::Vector2(5.0f, 5.0f));

	obstacle4.push_back(RVO::Vector2(5.0f, 5.0f));
	obstacle4.push_back(RVO::Vector2(5.0f, 6.0f));
	obstacle4.push_back(RVO::Vector2(-5.0f, 6.0f));
	obstacle4.push_back(RVO::Vector2(-5.0f, 5.0f));
	sim->addObstacle(obstacle1);
	sim->addObstacle(obstacle2);
	sim->addObstacle(obstacle3);
	sim->addObstacle(obstacle4);
	sim->processObstacles();

	sim->setTimeStep(timestep::dt);
	sim->setAgentDefaults(15.0f, 10, 2.0f, 2.0f, config::USER_RADIUS, config::VELOCITY);
	for (int i = 0; i < virt_env->get_obstacles().size()+1; i++) {
		goals.push_back(std::vector<RVO::Vector2>());
		history.push_back(std::vector<RVO::Vector2>());
	}
	// Get waypoints for path
	int numWaypoints = 43;
	for (int j = 0; j < numWaypoints; j++) {
		std::vector<RVO::Vector2> goalsSoFar;
		for (int i = 0; i < virt_env->get_obstacles().size() + 1; i++) {
			vec2f sampled_point = sample_point(virt_env, cur_virt_pos);
			auto sampledPoint = RVO::Vector2(sampled_point.x, sampled_point.y);
			while (!sampledPointOK(goalsSoFar, sampledPoint)) {
				sampled_point = sample_point(virt_env, cur_virt_pos);
				sampledPoint = RVO::Vector2(sampled_point.x, sampled_point.y);
			}
			goalsSoFar.push_back(sampledPoint);
		}
		// Add goals to the grand goals list
		for (int i = 0; i < virt_env->get_obstacles().size() + 1; i++) {
			goals[i].push_back(goalsSoFar[i]);
		}
		goalsSoFar.clear();
	}
	// Set initial positions for the agents
	for (int i = 0; i < virt_env->get_obstacles().size() + 1; i++) {
		// Set intial user position
		if (i == 0) {
			sim->addAgent(RVO::Vector2(cur_virt_pos.x, cur_virt_pos.y));
		}
		// Set initial obstacle position
		else {
			vec2f* p = virt_env->get_obstacles()[i-1]->pos;
			sim->addAgent(RVO::Vector2(p->x, p->y));
		}
	}
	std::cout << "got waypoints" << std::endl;

	// Run the RVO simulation
	float angle_per_dt = math::radians(config::ANGULAR_VELOCITY) / (1.0f / timestep::dt);
	std::vector<int> waypoint_timestamps;
	int count = 0;
	float cur_heading = cur_virt_heading;
	vec2f cur_pos = cur_virt_pos;
	for (int i = 0; i < numWaypoints; i++) {
		do {
			setPreferredVelocities(sim, goals, i);
			sim->doStep();
			recordHistory(sim, history);
			count++;
		} while (!reachedGoal(sim, goals, i));

		waypoint_timestamps.push_back(count);
		std::cout << "finished generating path for waypoint #" << i << std::endl;
		if (i == 5) {
			int t = 4;
		}
	}

	// Copy the RVO data over to pasumi
	int cur_waypoint_flag = 0;
	// Create the vectors to store the obstacle path data. will be sent to pasumi at the end.
	for (int j = 1; j < sim->getNumAgents(); j++) {
		new_obstacle_paths.push_back(std::vector<vec2f*>());
	}
	for (int i = 0; i < history[0].size(); i++) {
		vec2f next_pt = vec2f(history[0][i].x(), history[0][i].y());
		vec2f pos_to_point = normalize(next_pt - cur_pos);
		float heading_to_turn = signed_angle(rad_2_vec(cur_heading), pos_to_point);
		// Orient user to face next point
		int num_iter = math::abs(int(heading_to_turn / angle_per_dt));
		for (int j = 0; j < num_iter; j++) {
			cur_heading += angle_per_dt * math::sign(heading_to_turn);
			user_path.push_back(trajectory_unit(cur_pos.x, cur_pos.y, cur_heading));
		}
		float remainder = math::abs(heading_to_turn) - (angle_per_dt * num_iter);
		if (remainder) {
			cur_heading += remainder * math::sign(heading_to_turn);
			user_path.push_back(trajectory_unit(cur_pos.x, cur_pos.y, cur_heading));
			num_iter++;
		}
		// Walk the user to the next point
		user_path.push_back(trajectory_unit(next_pt.x, next_pt.y, cur_heading));
		cur_pos = next_pt;
		// Update the obstacles
		for (int j = 1; j < sim->getNumAgents(); j++) {
			for (int k = 0; k < num_iter; k++) {
				new_obstacle_paths[j - 1].push_back(new vec2f(history[j][i].x(), history[j][i].y()));
			}
		}
	}

	// Add the obstacle paths to the obstacles
	for (int j = 1; j < sim->getNumAgents(); j++) {
		virt_env->get_obstacles()[j - 1]->add_path(new_obstacle_paths[j - 1]);
	}
	std::cout << "done adding all paths to the obstacles"<< std::endl;
}

void test(int i) {


}

bool motion_model::sampledPointOK(std::vector<RVO::Vector2> goals, RVO::Vector2 pt) {
	for (auto p : goals) {
		if (RVO::abs(p - pt) < 1.5f) {
			return false;
		}
	}
	return true;
}

void motion_model::setPreferredVelocities(RVO::RVOSimulator* sim, std::vector<std::vector<RVO::Vector2>> goals, int curWaypoint) {
	/*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		RVO::Vector2 goalVector = goals[i][curWaypoint] - sim->getAgentPosition(i);

		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}

		sim->setAgentPrefVelocity(i, goalVector);

		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;

		//sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) + dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
	}
}

void motion_model::recordHistory(RVO::RVOSimulator* sim, std::vector<std::vector<RVO::Vector2>>& path_history) {
	for (int i = 0; i < sim->getNumAgents(); ++i) {
		path_history[i].push_back(sim->getAgentPosition(i));
	}
}

bool motion_model::reachedGoal(RVO::RVOSimulator* sim, std::vector<std::vector<RVO::Vector2>> goals, int curWaypoint) {
	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		// User's path is done, so just end the simulation here.
		if (RVO::abs(sim->getAgentPosition(i) - goals[i][curWaypoint]) < 0.5f && i == 0) {
		//if (RVO::abs(sim->getAgentPosition(i) - goals[i][curWaypoint]) < 1.0f && i == 0) {
			return true;
		}
		if (RVO::abs(sim->getAgentPosition(i) - goals[i][curWaypoint]) > 0.5f) {
		//if (RVO::abs(sim->getAgentPosition(i) - goals[i][curWaypoint]) > 1.0f) {
			return false;
		}
	}
	return true;
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

void motion_model::generate_trajectory(std::vector<trajectory_unit>& trajectory, float cur_virt_heading, vec2f cur_virt_pos, float angular_vel, float velocity) {
	switch (trajectory_model) {
	case TRAJECTORY_MODEL::STRAIGHT:
		generate_straight_trajectory(trajectory, cur_virt_heading, cur_virt_pos, angular_vel, velocity);
		break;
	case TRAJECTORY_MODEL::ROTATE:
		break;
	case TRAJECTORY_MODEL::SMOOTH:
		generate_smooth_trajectory(trajectory, cur_virt_heading, cur_virt_pos, angular_vel, velocity);
		break;
	}
}

void motion_model::generate_straight_trajectory(std::vector<trajectory_unit>& trajectory, float cur_virt_heading, vec2f cur_virt_pos, float angular_vel, float velocity) {
	/*
	vec2f cur_heading = rad_2_vec(cur_virt_heading);
	vec2f cur_pos = cur_virt_pos;

	for (int i = 0; i < cur_path.waypoints.size(); i++) {
		// Rotate to next waypoint
		float amount_to_rotate = signed_angle(cur_heading, normalize(cur_path.waypoints[i] - cur_pos));
		int num_turn_units = math::abs((math::degrees(amount_to_rotate) / angular_vel) / timestep::dt);
		for (int j = 0; j < num_turn_units; j++) {
			trajectory.push_back(trajectory_unit(0, 0, 1 * math::sign(amount_to_rotate)));
		}
		float remainder = (math::degrees(math::abs(amount_to_rotate)) - (num_turn_units * angular_vel * timestep::dt)) / (angular_vel * timestep::dt);
		if (remainder) trajectory.push_back(trajectory_unit(0, 0, math::abs(remainder) * math::sign(amount_to_rotate)));

		// Walk in a straight line to waypoint
		float amount_to_walk = length(cur_pos - cur_path.waypoints[i]);
		int num_walk_units = (amount_to_walk / velocity) / timestep::dt;
		for (int j = 0; j < num_walk_units; j++) {
			trajectory.push_back(trajectory_unit(1, 1, 0));
		}
		remainder = (math::abs(amount_to_walk) - (num_walk_units * velocity * timestep::dt)) / (velocity * timestep::dt);
		if (remainder) trajectory.push_back(trajectory_unit(remainder, remainder, 0));

		cur_heading = normalize(cur_path.waypoints[i] - cur_pos);
		cur_pos = cur_path.waypoints[i];
	}
	*/
}

void motion_model::generate_smooth_trajectory(std::vector<trajectory_unit>& trajectory, float cur_virt_heading, vec2f cur_virt_pos, float angular_vel, float velocity) {
	/*
	vec2f cur_heading = rad_2_vec(cur_virt_heading);
	vec2f cur_pos = cur_virt_pos;

	for (int i = 0; i < cur_path.waypoints.size(); i++) {
		// Rotate to next waypoint
		float amount_to_rotate = signed_angle(cur_heading, normalize(cur_path.waypoints[i] - cur_pos));
		int num_turn_units = math::abs((math::degrees(amount_to_rotate) / angular_vel) / timestep::dt);
		float amount_to_walk = length(cur_pos - cur_path.waypoints[i]);
		int num_walk_units = (amount_to_walk / velocity) / timestep::dt;

		int turn_counter = num_turn_units;
		int walk_counter = num_walk_units;
		while (turn_counter > 0 || walk_counter > 0) {
			if (turn_counter) {
				trajectory.push_back(trajectory_unit(0, 0, 1 * math::sign(amount_to_rotate)));
				turn_counter--;
			}
			else {
				float remainder = fmod(math::degrees(amount_to_rotate) / angular_vel, timestep::dt);
				if (remainder) trajectory.push_back(trajectory_unit(0, 0, math::abs(remainder) * math::sign(amount_to_rotate)));
			}

			if (walk_counter) {
				trajectory.push_back(trajectory_unit(1, 1, 0));
				walk_counter--;
			}
			else {
				float remainder = fmod(amount_to_walk / velocity, timestep::dt);
				if (remainder) trajectory.push_back(trajectory_unit(0, 0, remainder));
			}
		}

		cur_heading = normalize(cur_path.waypoints[i] - cur_pos);
		cur_pos = cur_path.waypoints[i];
	}
	*/
}