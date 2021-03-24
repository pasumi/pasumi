#pragma once

#include <vector>

#include "user.h"
#include "physical_environment.h"
#include "virtual_environment.h"
#include "timestep.h"
#include "simulation_state.h"

class simulation{
    public:
        simulation();
        simulation(std::vector<physical_environment*> physical_envs, virtual_environment* virtual_env, std::vector<user*> users);
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
        std::vector<physical_environment*> phys_envs;
        virtual_environment* virt_env;
};