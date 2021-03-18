#include "reset_to_gradient.h"
#include "config.h"

reset_to_gradient::reset_to_gradient() {
	name = "reset to gradient";
    for (int i = 0; i < GRADIENT_SAMPLE_RATE; i++) {
        theta_values.push_back(((2 * math::pi) / GRADIENT_SAMPLE_RATE) * i);
    }
}

reset_to_gradient::~reset_to_gradient() {

}

float reset_to_gradient::reset(simulation_state& sim_state, user* egocentric_user) {
    vec2f phys_heading = rad_2_vec(egocentric_user->state.get_phys_heading());
    float virt_heading = egocentric_user->state.get_virt_heading();
    vec2f phys_pos = egocentric_user->state.get_phys_pos();
    vec2f virt_pos = egocentric_user->state.get_virt_pos();
    std::vector<trajectory_unit>* path = &(egocentric_user->state.path);
    std::deque<proximity_container*> prox_queue = egocentric_user->proximity_queue;
    physical_environment* phys_env = egocentric_user->physical_env();
    vec2f phys_env_center = egocentric_user->physical_env()->get_center();
    float user_radius = egocentric_user->radius;

    vec2f gradient_dir = compute_gradient(phys_env, prox_queue, phys_pos);
    return reorient_to_target(phys_heading, virt_heading, phys_pos, virt_pos, gradient_dir + phys_pos, path);
}

vec2f reset_to_gradient::compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos) {
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

float reset_to_gradient::compute_repulsive_force(vec2f pos, std::vector<object*> objects) {
    float dist_sum = 0.0f;
    for (object* obj : objects) {
        if (obj->space == object::SPACE_TYPE::PHYS)
            dist_sum += 1.0f / obj->distance(pos);
    }
    return dist_sum;
}