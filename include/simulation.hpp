#pragma once

#include <vector>

#include "user.hpp"
#include "physical_environment.hpp"
#include "virtual_environment.hpp"
#include "timestep.h"
#include "simulation_state.h"

//struct /*simulation_state {
//    simulation_state() {}
//
//    simulation_state(std::vector<user*> users, physical_environment* phys_env, virtual_environment* virt_env) {
//        this->users = users;
//        this->phys_env = phys_env;
//        this->virt_env = virt_env;
//    }
//
//    std::vector<user*> get_other_users(user* relative_user) {
//        std::vector<user*> other_users;
//        for (user* u : users) {
//            if (u != relative_user) {
//                other_users.push_back(u);
//            }
//        }
//        return other_users;
//    }
//};*/

class simulation{
    public:
        simulation();
        simulation(std::vector<physical_environment*> physical_envs, virtual_environment* virtual_env, std::vector<user*> users);
        void add_user(user* usr);
        void step();
        void write();
        void render();
        void write(int frame_count, time_t start_time, time_t end_time);
        void check_simulation_parameters();
        void quit_early();

        bool done;
        std::vector<user*> users;
        simulation_state sim_state;
        
    private:
        void compute_clutter(physical_environment* p_env, virtual_environment* v_env);
        float get_environment_clutter(physical_environment* env);
        float get_environment_clutter(virtual_environment* env);

        std::vector<physical_environment*> phys_envs;
        virtual_environment* virt_env;
};