#pragma once

#include <vector>

#include "vec2f.h"
#include "virtual_environment.h"
#include "timestep.h"

/**
 * Struct that holds data describing the user's configuration (position and
 * orientation) in an environment.
 */
struct trajectory_unit {
	/**
	 * Default constructor.
	 */
	trajectory_unit() {
		x = 0.0f;
		y = 0.0f;
		theta = 0.0f;
	}

	/**
	 * Constructor that requires an (x, y) position and orientation.
	 * @param cur_x The x position of the user for this trajectory unit.
	 * @param cur_y The y position of the user for this trajectory unit.
	 * @param cur_theta The user's heading direction for this trajectory unit.
	 */
	trajectory_unit(float cur_x, float cur_y, float cur_theta) {
		x = cur_x;
		y = cur_y;
		theta = cur_theta;
	}

	/**
	 * Overload multiplication operator.
	 */
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

/**
 * Struct that holds data describing the user's path/trajectory 
 * in an environment.
 */
struct path {
	std::vector<trajectory_unit> waypoints;
};

/**
 * A class that handles generating paths for a user in an environment.
 * That is, this class contains the code that models the user's movement.
 * You need to use this class if you want to make a user walk through an environment.
 */
class motion_model {
	public:
		enum class PATH_MODEL { RANDOM, STRAIGHT, FILE };
		std::vector<char*> PATH_MODEL_STRINGS;

		/**
		 * Default constructor for the motion model class.
		 */
		motion_model();

		/**
		 * Constructor for the motion model class.
		 * @param waypoints The number of waypoints in each path.
		 * @param paths The number of paths in our simulation.
		 * @param path_model The path model we will use to generate paths.
		 */
		motion_model(int waypoints, int paths, PATH_MODEL path_model);

		/**
		 * Generate a path according to the motion model.
		 * @param user_path The vector of path steps (essentially, this is the path).
		 * @param virt_env The virtual environment in which we are generating paths.
		 * @param cur_virt_pos The current position in the virtual environment.
		 * @param cur_virt_heading The current heading in the virtual environment.
		 */
		void generate_path(std::vector<trajectory_unit>& user_path, virtual_environment* virt_env, vec2f cur_virt_pos, float cur_virt_heading);

		/**
		 * Gets the name of the path model being used, for logging purposes.
		 * @return The name of the path model.
		 */
		char* get_path_model_name();

	private:
		/**
		 * Generates a path from point A to point B according to the motion model.
		 * @param cur_pos The current position in the virtual environment.
		 * @param cur_heading The current heading in the virtual environment.
		 * @param goal_point The point we wish to generate a path to (point B).
		 * @param angle_per_dt The simulated user's angular velocity per timestep.
		 * @param meters_per_dt The simulated user's linear velocity per timestep.
		 * @param user_path The vector of path steps (essentially, this is the path).
		 */
		void generate_path_to_point(vec2f cur_pos, float& cur_heading, vec2f goal_point, float angle_per_dt, float meters_per_dt, std::vector<trajectory_unit>& user_path);

		/**
		 * Randomly samples a legal point in the environment. This point is used for
		 * the goal position in the path generation.
		 * @param virt_env The virtual environment from which we sample a point.
		 * @param start_pos The starting position from which we will travel to reach the
		 *					sampled point.
		 * @return The sampled point in the environment.
		 */
		vec2f sample_point(virtual_environment* virt_env, vec2f start_pos);

		/**
		 * Check if a straight line path has any intersections with obstacle 
		 * in the environment.
		 * @param env The environment that the path is in.
		 * @param p1 The first point of the path.
		 * @param p2 The second point of the path.
		 * @return True if the path intersects any obstacles, false otherwise.
		 */
		bool check_path_for_collision(environment* env, vec2f p1, vec2f p2);

		path cur_path; // Path the user is currently walking on.
		std::vector<path> all_paths; // All paths the user will walk on.
		PATH_MODEL path_model; // The model being used to generate user paths.
		int waypoints;
		int paths;
};