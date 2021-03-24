#pragma once

#include <vector>

#include "vec2f.h"
#include "virtual_environment.h"
#include "timestep.h"

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
};

struct path {
	//path() {};

	//std::vector<vec2f> waypoints;
	std::vector<trajectory_unit> waypoints;
};


class motion_model {
	public:
		enum class PATH_MODEL { RANDOM, STRAIGHT, FILE };
		std::vector<char*> PATH_MODEL_STRINGS;

		motion_model();
		motion_model(int waypoints, int paths, PATH_MODEL path_model);
		void generate_path(std::vector<trajectory_unit>& user_path, virtual_environment* virt_env, vec2f cur_virt_pos, float cur_virt_heading);
		char* get_path_model_name();

	private:
		void generate_path_to_point(vec2f cur_pos, float& cur_heading, vec2f goal_point, float angle_per_dt, float meters_per_dt, std::vector<trajectory_unit>& user_path);
		vec2f sample_point(virtual_environment* virt_env, vec2f start_pos);
		bool check_path_for_collision(environment* env, vec2f p1, vec2f p2);

		path cur_path;
		std::vector<path> all_paths;
		PATH_MODEL path_model;
		int waypoints;
		int paths;

		std::vector<std::vector<trajectory_unit>> rvo_paths;
};