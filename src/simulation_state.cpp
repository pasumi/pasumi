#include "simulation_state.h"

simulation_state::simulation_state() {

}

simulation_state::simulation_state(std::vector<user*> users, std::vector<physical_environment*> physical_envs, virtual_environment* virt_env) {
    this->users = users;
    this->phys_envs = physical_envs;
    this->virt_env = virt_env;
}

std::vector<user*> simulation_state::get_other_users(user* relative_user) {
    std::vector<user*> other_users;
    for (user* u : users) {
        if (u != relative_user) {
            other_users.push_back(u);
        }
    }
    return other_users;
}