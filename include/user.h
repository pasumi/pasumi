#pragma once

#include <deque>

// Forward declare
class user_state;
class simulation_state;

#include "vec2f.h"
#include "proximity_container.h"
#include "physical_environment.h"
#include "virtual_environment.h"
#include "obstacle.h"
#include "motion_model.h"
#include "redirector.h"
#include "history.h"

struct compare final {
    bool operator()(const proximity_container& lhs, const proximity_container& rhs) const noexcept {
        return lhs.get_distance() > rhs.get_distance();
    }
};

/**
 * Class for creating user objects. Users walk around in a virtual environment, and 
 * get steered around in their physical environment via a redirection controller. Most
 * of the action for the simulation happens in this class and user_state.cpp. The user
 * is represented as a circle with a specific radius.
 */
class user : public object {
    public:
        /**
         * Default constructor. Don't use this.
         */
        user();

        /**
         * Constructor for a user. Creating a user requires knowing the user's
         * position and orientation in both environments, pointers to these two environments, details about their path in the virtual environment, 
         * and a pointer to the redirection controller that will steer them 
         * around in the physical environment.
         * @param id An identifier number for the user
         * @param phys_start_pos The user's starting position in the physical 
                                 environment. If you have set the user's starting 
                                 position to be random in config.cpp, this value is 
                                 unused.
         * @param virt_start_pos The user's starting position in the virtual 
                                 environment. If you have set the user's starting 
                                 position to be random in config.cpp, this value is 
                                 unused.
         * @param phys_heading The user's starting heading in the physical 
                               environment. If you have set the user's starting 
                               heading to be random in config.cpp, this value is 
                               unused.
         * @param virt_heading The user's starting heading in the virtual 
                               environment. If you have set the user's starting 
                               heading to be random in config.cpp, this value is 
                               unused.
         * @param num_paths The amount of paths that the user will walk on over the
                            duration of the entire simulation.
         * @param num_waypoints The number of waypoints used to generate a user path.
         * @param path_model The model used to generate the user's path in the 
                             virtual environment.
         * @param rdw The redirection controller that the user will be steered with.
         * @param phys_env The physical environment that the user is in.
         * @param virt_env The virtual environment that the user is in.
         */
        user(int id, vec2f phys_start_pos, vec2f virt_start_pos, float phys_heading, float virt_heading, int num_paths, int num_waypoints, motion_model::PATH_MODEL path_model, redirector* rdw, environment* phys_env, environment* virt_env);

        /**
         * Advance the user's state in the simulation forward by one timestep.
         * @param sim_state The state of the simulation on the current timestep.
         */
        void step(simulation_state& sim_state);

        /**
         * Add the physical obstacles (including walls) to the user's proximity queue.
         * The proximity queue keeps track of the user's distance to obstacles, so
         * that we can do collision checks.
         * @param phys_env The physical environment that the user is located in.
         */
        void add_phys_env(physical_environment* phys_env);

        /**
         * Add the virtual obstacles (including walls) to the user's proximity queue.
         * The proximity queue keeps track of the user's distance to obstacles, so
         * that we can do collision checks.
         * @param virt_env The virtual environment that the user is located in.
         */
        void add_virt_env(virtual_environment* virt_env);

        /**
         * Add the other users to the user's proximity queue.
         * The proximity queue keeps track of the user's distance to obstacles, so
         * that we can do collision checks.
         * @param users A vector of all the users in the simulation.
         */
        void add_other_users(std::vector<user*> users);

        /**
         * Generate a path for the user in the virtual environment using their
         * assigned motion model.
         * @param virt_env The virtual environment that the user is located in.
         */
        void generate_motion(virtual_environment* virt_env);

        /**
         * Get the "wall" of the user that is closest to a given point. Since the
         * user is not actually an obstacle in the traditional sense, a "wall" is
         * actually just a wall of the imaginary bounding box of the user.
		 * @param p The point to which we wish to find the closest user "wall".
         * @return The normalized wall of the user's bounding box that is closest
                   to the point p.
         */
        vec2f get_closest_wall(vec2f p);

        /**
         * Get the environment obstacle (or user) that is closest to the user in
         * a specified environment (either physical or virtual).
         * @param space_to_search The environment in which we should retrieve the                            obstacle that is closest to the user.
         * @return The proximity container for the closest obstacle.
         */
        proximity_container* get_closest_obstacle(object::SPACE_TYPE space_to_search);

        /**
         * Log the user's path data to a file.
         * @param paths_increment The number of paths that have been completed since
                                  the last write. This is just a hack to make sure
                                  that the file name has the correct path number.
         */
        void write(int paths_increment);

        /**
         * Reset the user's state according to the configuration settings, to prepare
         * them for their next path that they will walk.
         * @param phys_env The physical environment that the user is in.
         * @param virt_env The virtual environment that the user is in.
         */
        void reset_state(environment* phys_env, environment* virt_env);

        /**
         * Set a flag so that we initiate a reset on the next timestep, instead of
         * having the user follow their virtual path as is normally done during 
         * timesteps.
         */
        void trigger_reset_on_next_step();

        /**
         * Initialize the user's state according to the given parameters and the
         * configuration file settings (config.cpp).
         * @param phys_env The physical environment that the user is in.
         * @param virt_env The virtual environment that the user is in.
         * @param phys_start_pos The user's starting position in the physical 
                                 environment. If you have set the user's starting 
                                 position to be random in config.cpp, this value is 
                                 unused.
         * @param virt_start_pos The user's starting position in the virtual 
                                 environment. If you have set the user's starting 
                                 position to be random in config.cpp, this value is 
                                 unused.
         * @param phys_heading The user's starting heading in the physical 
                               environment. If you have set the user's starting 
                               heading to be random in config.cpp, this value is 
                               unused.
         * @param virt_heading The user's starting heading in the virtual 
                               environment. If you have set the user's starting 
                               heading to be random in config.cpp, this value is 
                               unused.
         * @param num_paths The amount of paths that the user will walk on over the
                            duration of the entire simulation.
         * @param num_waypoints The number of waypoints used to generate a user path.
         * @param path_model The model used to generate the user's path in the 
                             virtual environment.
         * @param rdw The redirection controller that the user will be steered with.
         */
        void init_state(environment* phys_env, environment* virt_env, vec2f phys_start_pos, vec2f virt_start_pos, float phys_heading, float virt_heading, int num_paths, int num_waypoints, motion_model::PATH_MODEL path_model, redirector* rdw);

        /**
         * Set up the user's state data for the next path. Also log the data for
         * the path that the user just finished walking.
         * @param sim_state The state of the simulation on the current timestep.
         */
        void prep_for_next_path(simulation_state& sim_state);

        /**
         * Determine if the user is blocking a circle that follows a
         * straight-line segment path.
         * @param start The start point of the path.
         * @param end The end point of the path.
         * @param radius The radius of the circle that travels along the path.
         */
        bool is_blocking_path(vec2f start, vec2f end, float radius);

        /**
         * Doesn't actually get used for anything. Only here exists because user 
         * inherits from the object class, which requires this method to be 
         * implemented.
         * @param p The point from which the distance to the user is calculated.
         */
        float distance(vec2f p);

        /**
         * Get the name of the redirection controller that the user is being
         * steered by.
         */
        char* get_redirector_name();

        /**
         * Get the name of the reset heuristic that is used to reorient the
         * user after they get too close to an obstacle.
         */
        char* get_resetter_name();

        void print_pos();
        vec2f get_phys_pos();
        vec2f get_virt_pos();
        std::vector<vec2f*> get_vertices();
        physical_environment* get_phys_env();
        physical_environment* physical_env();
        virtual_environment* virtual_env();
        bool is_dynamic();

        int num_vertices;
        std::vector<vec2f> base_verts;
        int id;
        float radius;
        std::deque<proximity_container*> proximity_queue;
        std::vector<user*> other_users;
        user_state state;
        bool done;
        bool finished_path;
        int num_collisions;
        int paths_complete;
        redirector* rdw;
        
    private:
        /**
         * Check if there is a collision, whic occurs when the user is too close 
         * to an obstacle and they are not currently being reoriented via a reset.
         * @param sim_state The state of the simulation on the current timestep.
         */
        void handle_collisions(simulation_state& sim_state);

        /**
         * Send a signal to all the dynamic obstacles in the physical environment
         * to instruct them to stop moving. This is so that the obstacles don't
         * collide with the user while they are reorienting during a reset. 
         
         * This is basically unspecified behavior in the RDW literature, since there
         * is not much literature on experiements with dynamic non-user obstacles.
         * Thus, the behavior of the physical obstacles during the user's reset is
         * very much dependent on the specific application/scenario. It may be the 
         * case that stopping movement of the other physical obstacles is not what
         * you want; it just seemed like the most reasonable behavior to me.
         */
        void broadcast_pause();

        physical_environment* phys_env;
        virtual_environment* virt_env;
        history user_history;
        bool trigger_reset = false;
};