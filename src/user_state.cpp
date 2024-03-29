#include <iostream>
#include <cassert>

#include "user_state.h"
#include "math.hpp"
#include "timestep.h"
#include "config.h"

user_state::user_state(){

}

user_state::user_state(vec2f phys_pos, vec2f virt_pos, float phys_heading, float virt_heading, int num_paths, int num_waypoints, motion_model::PATH_MODEL path_model, redirector* rdw) {
    this->phys_heading = phys_heading;
    this->virt_heading = virt_heading;
    velocity = config::VELOCITY;
    this->phys_pos = phys_pos;
    this->virt_pos = virt_pos;
    this->model = motion_model(num_waypoints, num_paths, path_model);
    this->rdw = rdw;
    this->nav_state = NAVIGATION_STATE::OK;
    this->done = false;
    this->paths_complete = 0;
    this->angular_velocity = config::ANGULAR_VELOCITY;

    this->start_phys_pos = vec2f(phys_pos);
    this->start_virt_pos = vec2f(virt_pos);
    this->start_phys_heading = phys_heading;
    this->start_virt_heading = virt_heading;

    this->prev_phys_pos.x = phys_pos.x;
    this->prev_phys_pos.y = phys_pos.y;
    this->prev_phys_pos.theta = phys_heading;
    this->prev_virt_pos.x = virt_pos.x;
    this->prev_virt_pos.y = virt_pos.y;
    this->prev_virt_pos.theta = virt_heading;
    post_dynamic_obstacle_reset_timer = 0;
}

void user_state::generate_motion(virtual_environment* virt_env) {
    model.generate_path(path, virt_env, virt_pos, virt_heading);
}

void user_state::step(simulation_state& sim_state, user* egocentric_user) {
    if (done) return;

    manage_post_reset_timer();

    vec2f phys_pos_before_update = egocentric_user->state.get_phys_pos();
    float phys_heading_before_update = egocentric_user->state.get_phys_heading();
    vec2f virt_pos_before_update = egocentric_user->state.get_virt_pos();
    float virt_heading_before_update = egocentric_user->state.get_virt_heading();

    trajectory_unit cur_pos = path[0];

    float dx = cur_pos.x - prev_virt_pos.x;
    float dy = cur_pos.y - prev_virt_pos.y;
    float dpos = length(vec2f(prev_virt_pos.x, prev_virt_pos.y) - vec2f(cur_pos.x, cur_pos.y));
    float dtheta = signed_angle(rad_2_vec(prev_virt_pos.theta), rad_2_vec(cur_pos.theta));
    // Not resetting, so follow the path and apply redirection.
    if (nav_state != NAVIGATION_STATE::RESETTING) {
        redirection_unit redir = rdw->update(dx, dy, dtheta, sim_state, egocentric_user);

        // Update virtual state regardless of whatever redirection happens
        virt_heading = cur_pos.theta;
        virt_heading = fmod(virt_heading, math::two_pi);
        virt_pos = vec2f(cur_pos.x, cur_pos.y);

        // No redirection
        if (!redir.apply_rota && !redir.apply_trans &&
            !redir.apply_curve && !redir.apply_bend) {
            phys_heading += dtheta;
            float x_off = cos(phys_heading) * dpos;
            float y_off = sin(phys_heading) * dpos;
            phys_pos += vec2f(x_off, y_off);
        }
        // Redirection
        else {
            // Update heading
            if (dtheta) {
                phys_heading += dtheta * redir.rota_gain;
                phys_heading = fmod(phys_heading, math::pi * 2);
            }
            // Update position
            if (dpos) {
                // Scale the magnitude of the position change
                if (redir.apply_trans) {
                    dpos *= redir.trans_gain;
                }
                // Turn the physical heading before walking forward
                if (redir.apply_curve) {
                    phys_heading += timestep::dt * redir.curve_gain_dir * math::radians(redir.curve_gain);
                }
                // Walk forward (possibly scaled by translation gain, possibly 
                // redirected by curvature gain).
                float x_off = cos(phys_heading) * dpos;
                float y_off = sin(phys_heading) * dpos;
                phys_pos += vec2f(x_off, y_off);
            }
            if (redir.apply_bend) {
                // TODO: maybe one day.
            }
        }

        prev_redir = redir;
    }
    // Resetting, so just follow the path that was edited by the RDW controller.
    else {
        // Heading
        phys_heading += dtheta * rdw->resetting_gains.rota_gain;
        phys_heading = fmod(phys_heading, math::pi * 2);

        // Position
        float x_off = cos(phys_heading) * dpos;
        float y_off = sin(phys_heading) * dpos;
        phys_pos += vec2f(x_off, y_off);

        prev_redir = rdw->resetting_gains;
    }

    bool collision_happened = check_and_correct_collision(phys_pos_before_update, phys_pos, egocentric_user);

    // Collision is going to happen. Trigger a reset instead of 
    // going through with the position update.
    if (collision_happened || !egocentric_user->physical_env()->point_is_legal(egocentric_user->state.get_phys_pos())) {
        // Undo the position update that was just done above (when applying redirection)
        phys_pos = phys_pos_before_update;
        phys_heading = phys_heading_before_update;
        virt_pos = virt_pos_before_update;
        virt_heading = virt_heading_before_update;

        // Signal to force a reset (very slightly before it actually needs to happen 
        // according to the reset threshold, but is done since the simulation is 
        // going to miss said collision if we don't force the reset).
        egocentric_user->trigger_reset_on_next_step();
    }
    // No collision happened when simulating the next step, so we can keep going 
    // as normal.
    else {
        prev_virt_pos = cur_pos;

        assert(egocentric_user->physical_env()->point_is_legal(egocentric_user->state.get_phys_pos()) &&
            "The user is in an illegal physical state!");
        assert(sim_state.virt_env->point_is_legal(egocentric_user->state.get_virt_pos()) &&
            "The user is in an illegal virtual state!");

        path.erase(path.begin());
        if (!path.size()) {
            paths_complete++;
        }
    }
}

bool user_state::check_and_correct_collision(vec2f prev_phys_pos, vec2f cur_phys_pos, user* egocentric_user) {
    vec2f prev_pos = prev_phys_pos;
    vec2f cur_pos = cur_phys_pos;
    proximity_container* closest_obj = egocentric_user->get_closest_obstacle(object::SPACE_TYPE::PHYS);
    std::vector<vec2f*> closest_feature = closest_obj->get_closest_feature();

    // User-wall collision
    if (closest_feature.size() > 1) {
        return geom::line_line_intersect(&prev_pos, &cur_pos, closest_feature[0], closest_feature[1]);
    }
    // User-user collision
    else {
        // TODO: implement this
        return true;
        //trajectory_unit temp = ((user*)closest_obj->get_obstacle())->state.prev_phys_pos;
        //vec2f other_user_prev_phys_pos = vec2f(temp.x, temp.y);
        //vec2f other_user_cur_phys_pos = ((user*)closest_obj->get_obstacle())->state.get_phys_pos();
        //return geom::line_line_intersect(prev_pos, cur_pos, other_user_prev_phys_pos, other_user_cur_phys_pos);
    }
}

void user_state::set_position_and_heading(environment* phys_env, environment* virt_env) {
    // Reset position
    // Virtual position
    if (config::RANDOM_VIRT_START_POS) {
        vec2f rand_virt_start_pos = virt_env->sample_point();
        virt_pos = rand_virt_start_pos;
    }
    else {
        virt_pos = start_virt_pos;
    }
    // Physical position
    if (config::MATCH_PHYS_VIRT_POS) {
        phys_pos = virt_pos;
    }
    else if (config::RANDOM_PHYS_START_POS) {
        vec2f rand_phys_start_pos = phys_env->sample_point();
        phys_pos = rand_phys_start_pos;
    }
    else if (config::RADIUS_PHYS_START_POS) {
        vec2f rand_phys_start_pos = phys_env->sample_point();
        float dist_to_virt_pos = length(virt_pos - rand_phys_start_pos);

        while (dist_to_virt_pos > config::RADIUS_PHYS_START_POS_THRESHOLD) {
            rand_phys_start_pos = phys_env->sample_point();
            dist_to_virt_pos = length(virt_pos - rand_phys_start_pos);
        }

        phys_pos = rand_phys_start_pos;
    }
    else {
        phys_pos = start_phys_pos;
    }

    // Reset heading
    // Virtual heading
    if (config::RANDOM_VIRT_HEADING) {
        float rand_virt_heading = 0.0f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (math::two_pi - 0.0f)));
        virt_heading = rand_virt_heading;
    }
    else {
        virt_heading = start_virt_heading;
    }
    // Physical heading
    if (config::MATCH_PHYS_VIRT_HEADING) {
        phys_heading = virt_heading;
    }
    else {
        if (config::RANDOM_PHYS_HEADING) {
            float rand_phys_heading = 0.0f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (math::two_pi - 0.0f)));
            phys_heading = rand_phys_heading;
        }
        else {
            phys_heading = start_phys_heading;
        }
    }
}

void user_state::reset_state(environment* phys_env, environment* virt_env) {
    set_position_and_heading(phys_env, virt_env);

    this->prev_phys_pos.x = phys_pos.x;
    this->prev_phys_pos.y = phys_pos.y;
    this->prev_phys_pos.theta = phys_heading;
    this->prev_virt_pos.x = virt_pos.x;
    this->prev_virt_pos.y = virt_pos.y;
    this->prev_virt_pos.theta = virt_heading;

    this->nav_state = NAVIGATION_STATE::OK;
}

void user_state::manage_post_reset_timer() {
    switch (nav_state) {
        case NAVIGATION_STATE::RESETTING:
            if (rdw->reset_policy->reset_timer != 0) {
                rdw->reset_policy->reset_timer--;
            }
            else {
                nav_state = NAVIGATION_STATE::OK;
                just_finished_reset = true;
            }
            break;
        case NAVIGATION_STATE::OK:
            if (post_dynamic_obstacle_reset_timer > 0) {
                post_dynamic_obstacle_reset_timer--;
            }
            if (just_finished_reset) {
                just_finished_reset = false;
            }
            break;
    }
}
vec2f user_state::get_phys_pos() {
    return phys_pos;
}

vec2f user_state::get_virt_pos() {
    return virt_pos;
}

float user_state::get_phys_heading() {
    return phys_heading;
}

float user_state::get_virt_heading() {
    return virt_heading;
}

float user_state::get_velocity() {
    return velocity;
}

float user_state::get_angular_vel() {
    return angular_velocity;
}

void user_state::reset(simulation_state& sim_state, user* egocentric_user) {
    nav_state = NAVIGATION_STATE::RESETTING;
    rdw->reset(sim_state, egocentric_user);
}

trajectory_unit user_state::get_prev_move() {
    return prev_move;
}

trajectory_unit user_state::get_cur_move() {
    return path[0];
}

redirection_unit user_state::get_prev_redir() {
    return prev_redir;
}
