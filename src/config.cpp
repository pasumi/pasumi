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
	//vec2f center = vec2f(0.0f, 0.0f);
	physical_environment phys_env1 = physical_environment(
		// Boundary file
		"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/phys/boundaries/office_small.txt",

		// Obstacle file
		"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/phys/obstacles/office_small.txt",

		"phys env 1"
	);

	std::vector<physical_environment*> phys_envs = std::vector<physical_environment*>{
		&phys_env1
	};

	/////////////////////////////////////////////////////
	//////////////// VIRTUAL ENVIRONMENT ////////////////
	/////////////////////////////////////////////////////
	virtual_environment virt_env = virtual_environment(
		//Boundary file
		"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/virt/boundaries/office_small_truncated.txt", 

		// Obstacle file
		"C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/virt/obstacles/office_small_truncated.txt",

		"virt env 1"
	);

	/////////////////////////////////////////////////////
	////////////////////// RESETTER /////////////////////
	/////////////////////////////////////////////////////
	reset_to_forward_distance R2FD = reset_to_forward_distance();

	/////////////////////////////////////////////////////
	///////////////////// REDIRECTOR ////////////////////
	/////////////////////////////////////////////////////
	resetter* reset_policy = &R2FD;

	arc ARC = arc(&phys_env1, &virt_env, &R2FD);

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
		new user(1, vec2f(0.0f, 0.0f), vec2f(0.0f, 0.0f), math::pi / 2.0f, math::pi / 2.0f, NUM_PATHS, NUM_WAYPOINTS, motion_model::PATH_MODEL::RANDOM, &ARC, &phys_env1, &virt_env)
	};

	/////////////////////////////////////////////////////
	////////////////////// SETTINGS /////////////////////
	/////////////////////////////////////////////////////
	bool RANDOM_PHYS_START_POS = true;
	bool RADIUS_PHYS_START_POS = false;
	float RADIUS_PHYS_START_POS_THRESHOLD = 0.5f;
	bool RANDOM_VIRT_START_POS = true;
	bool MATCH_PHYS_VIRT_POS = false;
	bool RANDOM_PHYS_HEADING = true;
	bool RANDOM_VIRT_HEADING = true;
	bool MATCH_PHYS_VIRT_HEADING = false;

	bool GRAPHICS = false;
	bool DEBUG = false;
	int TRIAL_TO_DEBUG = 26; // This doesn't even work properly lol
	int SEED = 3226288;
	bool SEEDED = true;
	const char* DATA_DIR = "data";
}