#pragma once

#include <vector>

//#include "user.hpp"
class user;

#include "physical_environment.hpp"
#include "virtual_environment.hpp"

class simulation_state {
	public:
		simulation_state();
		simulation_state(std::vector<user*> users, std::vector<physical_environment*> physical_envs, virtual_environment* virt_env);
		std::vector<user*> get_other_users(user* relative_user);

		std::vector<user*> users;
		std::vector<physical_environment*> phys_envs;
		virtual_environment* virt_env;

	private:
};