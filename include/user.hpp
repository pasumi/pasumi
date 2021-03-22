#pragma once

#include <deque>

class user_state;

#include "vec2f.hpp"
#include "proximity_container.hpp"
#include "physical_environment.hpp"
#include "virtual_environment.hpp"
#include "obstacle.h"
#include "motion_model.h"
#include "redirector.h"
#include "history.h"

struct compare final {
    bool operator()(const proximity_container& lhs, const proximity_container& rhs) const noexcept {
        return lhs.get_distance() > rhs.get_distance();
    }
};

class simulation_state;

class user : public object {
    public:
        user();
        user(int id, vec2f phys_start_pos, vec2f virt_start_pos, float phys_heading, float virt_heading, int num_paths, int num_waypoints, motion_model::PATH_MODEL path_model, motion_model::TRAJECTORY_MODEL trajectory_model, redirector* rdw, environment* phys_env, environment* virt_env);
        void step(simulation_state& sim_state);
        void print_pos();
        void add_phys_env(physical_environment* phys_env);
        void add_virt_env(virtual_environment* virt_env);
        void generate_motion(virtual_environment* virt_env);
        vec2f get_phys_pos();
        vec2f get_virt_pos();
        std::vector<vec2f*> get_vertices();
        float distance(vec2f p);
        std::vector<vec2f> get_draw_vertices(float alpha);
        physical_environment* get_phys_env();
        bool is_blocking_path(vec2f start, vec2f end, float radius);
        vec2f get_closest_wall(vec2f p);
        proximity_container* get_closest_obstacle(object::SPACE_TYPE space_to_search);
        int vertex_buffer_size();
        int index_buffer_size();
        void write(int paths_increment);
        char* get_redirector_name();
        void reset_state(environment* phys_env, environment* virt_env);
        void add_other_users(std::vector<user*> users);
        void step_until_collision(float t);
        void trigger_reset_on_next_step();
        void init_state(environment* phys_env, environment* virt_env, vec2f phys_start_pos, vec2f virt_start_pos, float phys_heading, float virt_heading, int num_paths, int num_waypoints, motion_model::PATH_MODEL path_model, motion_model::TRAJECTORY_MODEL trajectory_model, redirector* rdw);
        void prep_for_next_path(simulation_state& sim_state);
        physical_environment* physical_env();
        virtual_environment* virtual_env();
        char* get_resetter_name();
        bool is_dynamic();

        int num_vertices;
        std::vector<vec2f> base_verts;
        std::vector<unsigned int> gl_indices;
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
        void handle_collisions(simulation_state& sim_state);
        void broadcast_pause();

        physical_environment* phys_env;
        virtual_environment* virt_env;
        history user_history;
        bool trigger_reset = false;
};