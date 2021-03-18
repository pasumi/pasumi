#include <cassert>
#include <tuple>
#include <iostream>

#include "apf_grad.h"
#include "math.hpp"
#include "config.h"

apf_grad::apf_grad(resetter* _resetter) {
    name = "APF Gradient based";
    cur_rota_gain = 0.5f;
    min_rota_gain = 0.67f;
    max_rota_gain = 1.24f;

    cur_trans_gain = 0.5f;
    min_trans_gain = 0.86f;
    max_trans_gain = 1.26f;

    curve_radius = 7.5f; // meters
    cur_curve_per_deg = curve_radius_to_deg_per_meter();
    curve_dir = 1; // curve to the left of the user == 1. curve to the right of the user == -1

    steer_target = vec2f(0.0f, 0.0f);
    gradient_dir = vec2f(0.0f, 0.0f);
    for (int i = 0; i < GRADIENT_SAMPLE_RATE; i++) {
        theta_values.push_back(((2 * math::pi) / GRADIENT_SAMPLE_RATE) * i);
    }

    cur_alignment = std::vector<float>{ 0.0f, 0.0f, 0.0f };
    reset_policy = _resetter;
}

redirection_unit apf_grad::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
    visibility_polygon cur_phys_vis_poly = get_current_visibility_polygon(egocentric_user->state.get_phys_pos(), (environment*)(egocentric_user->physical_env()));
    visibility_polygon cur_virt_vis_poly = get_current_visibility_polygon(egocentric_user->state.get_virt_pos(), sim_state.virt_env);

    update_losses(cur_phys_vis_poly, cur_virt_vis_poly, sim_state, egocentric_user);
    cur_alignment = std::vector<float>{ cur_north_loss, cur_east_loss, cur_west_loss };

    vec2f phys_pos = egocentric_user->state.get_phys_pos();
    std::deque<proximity_container*> prox_queue = egocentric_user->proximity_queue;
    trajectory_unit cur_move = egocentric_user->state.get_cur_move();
    float phys_heading = egocentric_user->state.get_phys_heading();

    gradient_dir = compute_gradient(egocentric_user->physical_env(), prox_queue, phys_pos);
    steer_target = phys_pos + gradient_dir;
    redirection_unit next_redirection = set_gains(dx,dy,dtheta, cur_move, phys_heading);
    return next_redirection;
}

visibility_polygon apf_grad::get_current_visibility_polygon(vec2f pos, environment* env) {
    return visibility_polygon(env, &pos);
}

void apf_grad::update_losses(visibility_polygon& phys_poly, visibility_polygon& virt_poly, simulation_state& sim_state, user* egocentric_user) {
    vec2f phys_heading = rad_2_vec(egocentric_user->state.get_phys_heading());
    vec2f virt_heading = rad_2_vec(egocentric_user->state.get_virt_heading());
    phys_poly.compute_nearest_features(phys_heading);
    virt_poly.compute_nearest_features(virt_heading);
    cur_north_loss = phys_poly.distance_north - virt_poly.distance_north;
    cur_east_loss = phys_poly.distance_east - virt_poly.distance_east;
    cur_west_loss = phys_poly.distance_west - virt_poly.distance_west;
    //cur_north_loss = math::abs(phys_poly.distance_north - virt_poly.distance_north);
    //cur_east_loss = math::abs(phys_poly.distance_east - virt_poly.distance_east);
    //cur_west_loss = math::abs(phys_poly.distance_west - virt_poly.distance_west);
}

vec2f apf_grad::compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos) {
    std::vector<object*> objects;
    for (proximity_container* c : prox_queue) {
        if (c->get_obstacle()->space == object::SPACE_TYPE::PHYS) {
            objects.push_back(c->get_obstacle());
        }
    }
    float cur_repulsive_force = compute_repulsive_force(phys_pos, objects);

    float min_force = std::numeric_limits<float>::max();
    vec2f min_force_pos;
    for (float theta : theta_values) {
        vec2f new_pos = phys_pos + (rad_2_vec(theta) * SFR2G_STEP_SIZE);
        if (!phys_env->point_is_legal(new_pos)) continue;

        float new_force = compute_repulsive_force(new_pos, objects);
        if (new_force <= min_force) {
            min_force = new_force;
            min_force_pos = new_pos;
        }
    }

    assert(min_force != std::numeric_limits<float>::max());
    return normalize(min_force_pos - phys_pos);
}

float apf_grad::compute_repulsive_force(vec2f pos, std::vector<object*> objects) {
    float dist_sum = 0.0f;
    for (object* obj : objects) {
        if (obj->space == object::SPACE_TYPE::PHYS)
            dist_sum += 1.0f / obj->distance(pos);
    }
    return dist_sum;
}

redirection_unit apf_grad::set_gains(float dx, float dy, float dtheta, trajectory_unit cur_move, float phys_heading) {
    //trajectory_unit next_move = cur_move;
    vec2f user_dir = rad_2_vec(phys_heading);
    float angle_to_gradient = signed_angle(user_dir, gradient_dir);
    redirection_unit redir_unit = redirection_unit();

    // User is rotating
    if (dtheta) {
        // User has to turn right to face the gradient
        if (math::sign(angle_to_gradient) == -1) {
            // User is turning to the right
            if (dtheta < 0) {
                redir_unit.rota_gain = max_rota_gain;
                cur_rota_gain = max_rota_gain;
            }
            // User is turning to the left
            else {
                redir_unit.rota_gain = min_rota_gain;
                cur_rota_gain = min_rota_gain;
            }
        }
        // User has to turn left to face the gradient
        else {
            // User is turning to the right
            if (dtheta < 0) {
                redir_unit.rota_gain = min_rota_gain;
                cur_rota_gain = min_rota_gain;
            }
            // User is turning to the left
            else {
                redir_unit.rota_gain = max_rota_gain;
                cur_rota_gain = max_rota_gain;
            }
        }

        redir_unit.apply_rota = true;
        redir_unit.apply_trans = false;
        redir_unit.apply_curve = false;
        redir_unit.apply_bend = false;
    }
    // User is translating
    else if (dx || dy) {
        // User has to turn right to face the gradient
        if (math::sign(angle_to_gradient) == -1) {
            redir_unit.curve_gain = cur_curve_per_deg;
            redir_unit.curve_gain_dir = -1;
            curve_dir = -1;
        }
        // User has to turn left to face the gradient
        else {
            redir_unit.curve_gain = cur_curve_per_deg;
            redir_unit.curve_gain_dir = 1;
            curve_dir = 1;
        }

        // User is facing away from the gradient, slow them down
        if (dot(user_dir, gradient_dir) < 0) {
            redir_unit.trans_gain = min_trans_gain;
            cur_trans_gain = min_trans_gain;
        }
        // The paper says nothing about what translation gain is applied when the dot
        // product is not negative...
        else {
            redir_unit.trans_gain = 1.0f;
            cur_trans_gain = 1.0f;
        }

        redir_unit.apply_rota = false;
        redir_unit.apply_trans = true;
        redir_unit.apply_curve = true;
        redir_unit.apply_bend = false;
    }

    return redir_unit;
}


std::vector<std::vector<vec2f>> apf_grad::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
    std::vector<std::vector<vec2f>> all;
    for (float i = -4.9f; i < 5.0f; i += 0.1f) {
        std::vector<vec2f> vec;
        for (float j = -4.9f; j < 5.0f; j += 0.1f) {
            vec2f p = vec2f(i, j);
            if (p.x > -1.9f && p.x < 1.7f && p.y < -2.1f && p.y > -2.3f) {
                int f = 4;
            }
            if (phys_env->point_is_legal(p)) {
                vec2f temp = (compute_gradient(phys_env, prox_queue, p));
                vec.push_back(normalize(compute_gradient(phys_env, prox_queue, p)));
            }
            else {
                vec.push_back(vec2f(-10.0f, -10.0f));
            }
        }
        all.push_back(vec);
    }
    gradient_data = all;
    return all;
}

