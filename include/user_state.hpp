#pragma once

#include <deque>

class user;

#include "vec2f.hpp"
#include "motion_model.h"
#include "timestep.h"
#include "redirector.h"
#include "physical_environment.hpp"
#include "proximity_container.hpp"
#include "simulation_state.h"
#include "motion_model.h"

class user_state{
    public:
        enum class NAVIGATION_STATE { OK, RESETTING };

        user_state();
        user_state(vec2f phys_pos, vec2f virt_pos, float phys_heading, float virt_heading, int num_paths, int num_waypoints, motion_model::PATH_MODEL path_model, motion_model::TRAJECTORY_MODEL trajectory_model, redirector* rdw);
        void step(simulation_state& sim_state, user* egocentric_user);
        float get_phys_heading();
        float get_virt_heading();
        vec2f get_phys_pos();
        vec2f get_virt_pos();
        float get_velocity();
        float get_angular_vel();
        void generate_motion(virtual_environment* virt_env);
        void reset(simulation_state& sim_state, user* egocentric_user);
        trajectory_unit get_prev_move();
        trajectory_unit get_cur_move();
        redirection_unit get_prev_redir();
        void manage_post_reset_timer();
        void reset_state(environment* phys_env, environment* virt_env);
        void set_position_and_heading(environment* phys_env, environment* virt_env);
        vec2f get_next_phys_pos(float seconds_into_future);
        vec2f get_next_virt_pos(float seconds_into_future);
        bool check_and_correct_collision(vec2f prev_phys_pos, vec2f cur_phys_pos, user* egocentric_user);
        std::vector<float> get_alignment();

        NAVIGATION_STATE nav_state;
        std::vector<trajectory_unit> path;
        motion_model model;
        redirector* rdw;
        bool done;
        int paths_complete;
        bool just_finished_reset;

        trajectory_unit prev_phys_pos;
        trajectory_unit prev_virt_pos;
        int post_dynamic_obstacle_reset_timer;

    private:
        vec2f phys_pos;
        vec2f virt_pos;
        float phys_heading;  // Radians
        float virt_heading;  // Radians
        float angular_velocity; // deg/s
        float velocity; // m/s
        trajectory_unit prev_move; // for rendering intepolation purposes
        redirection_unit prev_redir;

        vec2f start_phys_pos;
        vec2f start_virt_pos;
        float start_phys_heading;  // Radians
        float start_virt_heading;  // Radians
};