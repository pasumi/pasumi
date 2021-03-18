#pragma once

#include "user.hpp"
#include "user_state.hpp"
#include "vec2f.hpp"
#include "redirector.h"
#include "apf_grad.h"
#include "apf_vec.h"
#include "no_rdw.h"
#include "obstacle.h"
#include "math.hpp"
#include "arc.h"
#include "s2c.h"
#include "arc_simple.h"
#include "modified_reset_to_center.h"
#include "reset_to_gradient.h"
#include "step_forward_reset_to_gradient.h"
#include "reset_to_forward_distance.h"
#include "resetter.h"
#include "more_rays.h"

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
	extern physical_environment phys_env2;
	extern std::vector<physical_environment*> phys_envs;

	/////////////////////////////////////////////////////
	//////////////// VIRTUAL ENVIRONMENT ////////////////
	/////////////////////////////////////////////////////
	extern virtual_environment virt_env;

	/////////////////////////////////////////////////////
	////////////////////// RESETTER /////////////////////
	/////////////////////////////////////////////////////
	extern modified_reset_to_center MR2C;
	extern reset_to_gradient R2G;
	extern step_forward_reset_to_gradient SFR2G;
	extern reset_to_forward_distance R2FD;

	/////////////////////////////////////////////////////
	///////////////////// REDIRECTOR ////////////////////
	/////////////////////////////////////////////////////
	extern int POST_RESET_GRACE_PERIOD;
	//extern apf_grad APF_GRAD1;
	extern bool APF_USE_TRANS;
	extern arc ARC_V2;
	extern more_rays MORE_RAYS;

	/////////////////////////////////////////////////////
	//////////////////// MOTION MODEL ///////////////////
	/////////////////////////////////////////////////////
	extern float MIN_WAYPOINT_DISTANCE; // Minimum distance between two waypoints 
	extern float MAX_WAYPOINT_DISTANCE; // Maximum distance between two waypoints
	extern float MIN_WAYPOINT_ANGLE; // Minimum angle between two waypoints
	extern float MAX_WAYPOINT_ANGLE; // Maximum angle between two waypoints
	extern std::string path_file;

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
	extern const char* DATA_DIR;
}