#pragma once

#include "user.h"
#include "user_state.h"
#include "vec2f.h"
#include "redirector.h"
#include "obstacle.h"
#include "math.hpp"
#include "arc.h"
#include "reset_to_forward_distance.h"
#include "resetter.h"

namespace config {
	/////////////////////////////////////////////////////
	/////////////// PHYSICAL ENVIRONMENT ////////////////
	/////////////////////////////////////////////////////
	/*** VERTICES ***/
	extern vec2f center;
	extern std::vector<vec2f> phys_verts;

	/*** OBSTACLES ***/
	extern std::vector<obstacle*> obstacles;

	extern physical_environment phys_env1;
	extern std::vector<physical_environment*> phys_envs;

	/////////////////////////////////////////////////////
	//////////////// VIRTUAL ENVIRONMENT ////////////////
	/////////////////////////////////////////////////////
	extern virtual_environment virt_env;

	/////////////////////////////////////////////////////
	////////////////////// RESETTER /////////////////////
	/////////////////////////////////////////////////////
	extern reset_to_forward_distance R2FD;

	/////////////////////////////////////////////////////
	///////////////////// REDIRECTOR ////////////////////
	/////////////////////////////////////////////////////
	extern int POST_RESET_GRACE_PERIOD;

	/////////////////////////////////////////////////////
	//////////////////// MOTION MODEL ///////////////////
	/////////////////////////////////////////////////////
	// These waypoint distance parameters are used in the Azmandian path model.
	extern float MIN_WAYPOINT_DISTANCE; // Minimum distance between two waypoints 
	extern float MAX_WAYPOINT_DISTANCE; // Maximum distance between two waypoints
	extern float MIN_WAYPOINT_ANGLE;	// Minimum angle between two waypoints
	extern float MAX_WAYPOINT_ANGLE;	// Maximum angle between two waypoints
	extern fs::path path_file;

	/////////////////////////////////////////////////////
	/////////////////////// USERS ///////////////////////
	/////////////////////////////////////////////////////
	extern float DISTANCE_THRESHOLD;
	extern float RESET_DISTANCE_CHECK_VALUE;
	extern float USER_RADIUS;
	extern int NUM_PATHS;
	extern int NUM_WAYPOINTS;
	extern float RADIUS;
	extern float VELOCITY;
	extern float ANGULAR_VELOCITY;
	extern std::vector<user*> users;

	/////////////////////////////////////////////////////
	////////////////////// SETTINGS /////////////////////
	/////////////////////////////////////////////////////
	extern bool RANDOM_PHYS_START_POS;
	extern bool RADIUS_PHYS_START_POS;
	extern float RADIUS_PHYS_START_POS_THRESHOLD;
	extern bool RANDOM_VIRT_START_POS;
	extern bool MATCH_PHYS_VIRT_POS;
	extern bool RANDOM_PHYS_HEADING;
	extern bool RANDOM_VIRT_HEADING;
	extern bool MATCH_PHYS_VIRT_HEADING;
	extern bool GRAPHICS;
	extern bool DEBUG;
	extern int TRIAL_TO_DEBUG;
	extern int SEED;
	extern bool SEEDED;
	extern const fs::path DATA_DIR;
}