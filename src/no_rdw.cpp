#include "no_rdw.h"
#include "config.h"

no_rdw::no_rdw(resetter* _resetter) {
    name = "No redirection";
	steer_target = vec2f(0.0f, 0.0f);
	gradient_dir = vec2f(0.0f, 0.0f);
    for (int i = 0; i < GRADIENT_SAMPLE_RATE; i++) {
        theta_values.push_back(((2 * math::pi) / GRADIENT_SAMPLE_RATE) * i);
    }

	reset_policy = _resetter;
}

redirection_unit no_rdw::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	redirection_unit redir_unit = redirection_unit();
	return redir_unit;
}

std::vector<std::vector<vec2f>> no_rdw::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
	std::vector<std::vector<vec2f>> test;
	return test;
}

float no_rdw::compute_repulsive_force(vec2f pos, std::vector<object*> objects) {
    float dist_sum = 0.0f;
    for (object* obj : objects) {
        if (obj->space == object::SPACE_TYPE::PHYS)
            dist_sum += 1.0f / obj->distance(pos);
    }
    return dist_sum;
}

vec2f no_rdw::compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos) {
    std::vector<object*> objects;
    for (proximity_container* c : prox_queue) {
        objects.push_back(c->get_obstacle());
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