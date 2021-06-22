#pragma once

#include <vector>

// Forward declare
class user;

#include "physical_environment.h"
#include "virtual_environment.h"

/**
 * Class that describes the state of the simulation. This class creates an object
 * that holds a pointer to the environments and users present in the simulation.
 * This is useful when we need to get information about the VR system (environments
 * and users) in our redirection controller.
 */
class simulation_state {
	public:
		/**
		 * Default constructor. Don't use this.
		 */
		simulation_state();

		/**
		 * Constructor that accepts pointers to the VR system data objects.
		 * @param users A vector of user objects currently walking in the simulation.
		 * @param phys_envs A vector of the physical environments that users are in.
		 * @param virt_env The virtual environment that the users are in. (Remeber 
						   that pasumi currently only supports one virtual 
						   environment)
		 */
		simulation_state(std::vector<user*> users, std::vector<physical_environment*> physical_envs, virtual_environment* virt_env);

		/**
		 * Get a list of all users except the specified user. This is useful
		 * when we want to know the locations of users relative to a given user,
		 * usually so that we can steer that given user away from the others.
		 * @param relative_user The user that should be excluded from the returned 
								list of users.
		 * @return A vector containing all user objects except the user passed in 
				   as a parameter.
		 */
		std::vector<user*> get_other_users(user* relative_user);

		std::vector<user*> users;
		std::vector<physical_environment*> phys_envs;
		virtual_environment* virt_env;

	private:
};