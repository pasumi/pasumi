#include <limits>
#include <chrono>

#include "config.h"

int check_settings() {
	if (config::SEEDED) {
		srand(static_cast <unsigned> (config::SEED));
	}
	else {
		int unseeded = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
			).count(); // Need millisecond precision
		srand(unseeded);
	}
	rand(); // Do this to avoid a bug on Windows where the first value is not random, but instead steadily increases.

	assert(!(config::RANDOM_PHYS_START_POS && config::RADIUS_PHYS_START_POS) &&
		   "Cannot have a random physical start position and a radius-based start position.");

	if (config::MATCH_PHYS_VIRT_HEADING) {
		assert(config::RANDOM_PHYS_HEADING == config::RANDOM_VIRT_HEADING);
	}

	return 0;
}

namespace config {
	int _ = check_settings();

	/////////////////////////////////////////////////////
	/////////////// PHYSICAL ENVIRONMENT ////////////////
	/////////////////////////////////////////////////////
	/*** VERTICES ***/
	vec2f center = vec2f(0.0f, 0.0f);
	std::vector<vec2f> phys_verts = std::vector<vec2f>{
		/*vec2f(0.0f, 3.53553391f),
		vec2f(3.53553391f, 0.0f),
		vec2f(0.0f, -3.53553391f),
		vec2f(-3.53553391f, 0.0f)*/

		vec2f(5.0f, -5.0f),
		vec2f(5.0f, 5.0f),
		vec2f(-5.0f, 5.0f),
		vec2f(-5.0f, -5.0f)
	};

	/*** OBSTACLES ***/
	std::vector<obstacle*> phys_obstacles = std::vector<obstacle*>{
	};

	//physical_environment phys_env = physical_environment(phys_verts, phys_obstacles, center);
	physical_environment phys_env1 = physical_environment(
		// Boundary file
		"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/phys/boundaries/small_square.txt",

		// Obstacle file
		"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/phys/obstacles/ismar_test.txt",

		"phys env 1"
	);

	//physical_environment phys_env2 = physical_environment(
	//	// Boundary file
	//	"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/phys/boundaries/office_small_truncated.txt",

	//	// Obstacle file
	//	"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/phys/obstacles/office_small_truncated.txt"
	//);

	std::vector<physical_environment*> phys_envs = std::vector<physical_environment*>{
		&phys_env1
	};

	/////////////////////////////////////////////////////
	//////////////// VIRTUAL ENVIRONMENT ////////////////
	/////////////////////////////////////////////////////
	virtual_environment virt_env = virtual_environment(
		//Boundary file
		"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/virt/boundaries/small_square.txt", 

		// Obstacle file
		"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/virt/obstacles/empty.txt",

		"virt env 1"
	);

	/////////////////////////////////////////////////////
	////////////////////// RESETTER /////////////////////
	/////////////////////////////////////////////////////
	modified_reset_to_center MR2C = modified_reset_to_center();
	reset_to_gradient R2G = reset_to_gradient();
	step_forward_reset_to_gradient SFR2G = step_forward_reset_to_gradient();
	reset_to_forward_distance R2FD = reset_to_forward_distance();

	/////////////////////////////////////////////////////
	///////////////////// REDIRECTOR ////////////////////
	/////////////////////////////////////////////////////
	//resetter* reset_policy = &SFR2G;
	//resetter* reset_policy = &MR2C;
	resetter* reset_policy = &R2FD;

	int POST_RESET_GRACE_PERIOD = 5;
	apf_grad APF_GRAD1 = apf_grad(reset_policy);
	apf_grad APF_GRAD2 = apf_grad(reset_policy);
	apf_vec APF_VEC1 = apf_vec(&phys_env1, reset_policy);
	s2c S2C = s2c(reset_policy);
	arc_simple ARC_V1 = arc_simple(&phys_env1, &virt_env, reset_policy);
	arc ARC_V2 = arc(&phys_env1, &virt_env, reset_policy);
	more_rays MORE_RAYS = more_rays(&phys_env1, &virt_env, reset_policy);
	no_rdw NO_RDW1 = no_rdw(reset_policy);
	bool APF_USE_TRANS = false;

	/////////////////////////////////////////////////////
	//////////////////// MOTION MODEL ///////////////////
	/////////////////////////////////////////////////////
	float MIN_WAYPOINT_DISTANCE = 2.0f;
	float MAX_WAYPOINT_DISTANCE = 6.0f;
	float MIN_WAYPOINT_ANGLE = -math::pi; 
	float MAX_WAYPOINT_ANGLE = math::pi; 
	std::string path_file = "C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/path.txt";

	/////////////////////////////////////////////////////
	/////////////////////// USERS ///////////////////////
	/////////////////////////////////////////////////////

	float DISTANCE_THRESHOLD = 0.05f;
	float RESET_DISTANCE_CHECK_VALUE = config::DISTANCE_THRESHOLD + (timestep::dt * 3) + config::USER_RADIUS;
	float USER_RADIUS = 0.5f;
	int NUM_PATHS = 100;
	int NUM_WAYPOINTS = 100;
	float VELOCITY = 1.0f;
	float ANGULAR_VELOCITY = 90.0f;
	std::vector<user*> users = std::vector<user*>{
		new user(1, vec2f(0.15f, 0.0f), vec2f(0.0f, 0.0f), math::pi / 2.0f, math::pi / 2.0f, NUM_PATHS, NUM_WAYPOINTS, motion_model::PATH_MODEL::FILE, motion_model::TRAJECTORY_MODEL::STRAIGHT, &ARC_V2, &phys_env1, &virt_env)
		//,

		//new user(2, vec2f(0.0f, -3.5f), vec2f(0.0f, -3.5f), math::pi / 2.0f, math::pi / 2.0f, 2, 4, motion_model::PATH_MODEL::RANDOM, motion_model::TRAJECTORY_MODEL::STRAIGHT, &S2C, &phys_env2, &virt_env)
	};




	/////////////////////////////////////////////////////
	////////////////////// SETTINGS /////////////////////
	/////////////////////////////////////////////////////
		/*
	bool RANDOM_PHYS_START_POS = true;
	bool RADIUS_PHYS_START_POS = false;
	float RADIUS_PHYS_START_POS_THRESHOLD = 0.5f;
	bool RANDOM_VIRT_START_POS = true;
	bool MATCH_PHYS_VIRT_POS = false;
	bool RANDOM_PHYS_HEADING = true;
	bool RANDOM_VIRT_HEADING = true;
	bool MATCH_PHYS_VIRT_HEADING = false;
	*/

	bool RANDOM_PHYS_START_POS = false;
	bool RADIUS_PHYS_START_POS = false;
	float RADIUS_PHYS_START_POS_THRESHOLD = 0.5f;
	bool RANDOM_VIRT_START_POS = false;
	bool MATCH_PHYS_VIRT_POS = false;
	bool RANDOM_PHYS_HEADING = false;
	bool RANDOM_VIRT_HEADING = false;
	bool MATCH_PHYS_VIRT_HEADING = false;

	bool GRAPHICS = true;
	bool DEBUG = false;
	int TRIAL_TO_DEBUG = 26; // This doesn't even work properly lol
	int SEED = 3226288;
	bool SEEDED = true;
	const char* DATA_DIR = "data";
}