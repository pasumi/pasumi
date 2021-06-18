#pragma once

#include <vector>

#include "user.h"
#include "physical_environment.h"
#include "virtual_environment.h"
#include "timestep.h"
#include "simulation_state.h"

/**
 * Class for a simulation object. This class describes and keeps track of the entire 
 * simulation of users walking around in VR. This class doesn't actually do that much,
 * since most of the time integration and collision-checking happens in the user.h 
 * class. Most of this class is just responsible for advancing the timestep and 
 * dealing with bookkeeping things like logging.
 * 
 * Currently, the simulation only supports users in the same virtual environment. 
 * However, these users can be located in separate physical environments.
 */
class simulation{
    public:
        /**
         * Default constructor. Don't use this.
         */
        simulation();

        /**
         * Constructor. Requires the physical and virtual environments, as well as
         * the user(s) in these environments.
         * @param physical_envs The physical environments that the user(s) are located
                                in.
         * @param virtual_env The virtual environment that the users are located in.
         * @param users All of the users who are walking around in the simulation.
         */
        simulation(std::vector<physical_environment*> physical_envs, virtual_environment* virtual_env, std::vector<user*> users);

        /**
         * Advance the simulation by one timestep.
         */
        void step();

        /**
         * Log the information about the simulation to a file.
         * @param frame_count The number of frames elapsed during the simulation.
         * @param start_time When the simulation started.
         * @param end_time When the simulation ended.
         */
        void write(int frame_count, time_t start_time, time_t end_time);

        /**
         * Makes sure that the simulation settings from config.cpp are legal.
         */
        void check_simulation_parameters();

        /**
         * Allows the user to interrupt the simulation early (via ctrl+C in the 
         * terminal). This method just makes sure that the data from the interrupted 
         * path is logged in a file.
         */
        void quit_early();

        bool done;
        std::vector<user*> users;
        simulation_state sim_state; // Pointers to the users and environments.
        
    private:
        std::vector<physical_environment*> phys_envs;
        virtual_environment* virt_env;
};