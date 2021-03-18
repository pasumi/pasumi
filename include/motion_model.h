#pragma once

#include <vector>

#include "vec2f.hpp"
#include "virtual_environment.hpp"
#include "timestep.h"
#include "include/rvo/RVO.h"

/*
struct trajectory_unit {
	trajectory_unit() {
		dx = 0.0f;
		dy = 0.0f;
		dtheta = 0.0f;
	}

	trajectory_unit(float x, float y, float theta) {
		dx = x;
		dy = y;
		dtheta = theta;
	}

	struct trajectory_unit& operator*(const float& rhs) {
		dx *= rhs;
		dy *= rhs;
		dtheta *= rhs;
		return *this;
	}

	float dx;
	float dy;
	float dtheta;
};
*/

struct trajectory_unit {
	trajectory_unit() {
		x = 0.0f;
		y = 0.0f;
		theta = 0.0f;
	}

	trajectory_unit(float cur_x, float cur_y, float cur_theta) {
		x = cur_x;
		y = cur_y;
		theta = cur_theta;
		//from_reset = false;
	}

	struct trajectory_unit& operator*(const float& rhs) {
		x *= rhs;
		y *= rhs;
		theta *= rhs;
		return *this;
	}

	float x;
	float y;
	float theta;
	//bool from_reset;
};

struct path {
	//path() {};

	//std::vector<vec2f> waypoints;
	std::vector<trajectory_unit> waypoints;
};


class motion_model {
	public:
		enum class PATH_MODEL { RANDOM, STRAIGHT, AZMANDIAN, TEST, FILE, RVO };
		std::vector<char*> PATH_MODEL_STRINGS;
		enum class TRAJECTORY_MODEL { STRAIGHT, ROTATE, SMOOTH };
		std::vector<char*> TRAJECTORY_MODEL_STRINGS;

		motion_model();
		motion_model(int waypoints, int paths, PATH_MODEL path_model, TRAJECTORY_MODEL trajectory_model);
		void generate_path(std::vector<trajectory_unit>& user_path, virtual_environment* virt_env, vec2f cur_virt_pos, float cur_virt_heading);
		void generate_trajectory(std::vector<trajectory_unit>& trajectory, float cur_virt_heading, vec2f cur_virt_pos, float angular_vel, float velocity);
		void generate_straight_trajectory(std::vector<trajectory_unit>& trajectory, float cur_virt_heading, vec2f cur_virt_pos, float angular_vel, float velocity);
		void generate_smooth_trajectory(std::vector<trajectory_unit>& trajectory, float cur_virt_heading, vec2f cur_virt_pos, float angular_vel, float velocity);
		char* get_path_model_name();
		char* get_trajectory_model_name();

	private:
		void generate_path_to_point(vec2f cur_pos, float& cur_heading, vec2f goal_point, float angle_per_dt, float meters_per_dt, std::vector<trajectory_unit>& user_path);
		vec2f sample_point(virtual_environment* virt_env, vec2f start_pos);
		bool check_path_for_collision(environment* env, vec2f p1, vec2f p2);
		void load_RVO_data();
		void get_RVO_path(std::vector<trajectory_unit>& user_path, virtual_environment* virt_env, vec2f cur_virt_pos, float cur_virt_heading);
		void setPreferredVelocities(RVO::RVOSimulator* sim, std::vector<std::vector<RVO::Vector2>> goals, int curWaypoint);
		void recordHistory(RVO::RVOSimulator* sim, std::vector<std::vector<RVO::Vector2>>& path_history);
		bool reachedGoal(RVO::RVOSimulator* sim, std::vector<std::vector<RVO::Vector2>> goals, int curWaypoint);
		bool sampledPointOK(std::vector<RVO::Vector2> goals, RVO::Vector2 pt);

		path cur_path;
		std::vector<path> all_paths;
		PATH_MODEL path_model;
		TRAJECTORY_MODEL trajectory_model;
		int waypoints;
		int paths;

		std::vector<std::vector<trajectory_unit>> rvo_paths;
};