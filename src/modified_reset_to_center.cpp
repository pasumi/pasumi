#include "modified_reset_to_center.h"
#include "config.h"

modified_reset_to_center::modified_reset_to_center() {
	name = "modified reset to center";
}

modified_reset_to_center::~modified_reset_to_center() {

}

float modified_reset_to_center::reset(simulation_state& sim_state, user* egocentric_user) {
    object* closest_object = egocentric_user->get_closest_obstacle(object::SPACE_TYPE::PHYS)->get_obstacle();

    vec2f phys_heading = rad_2_vec(egocentric_user->state.get_phys_heading());
    float virt_heading = egocentric_user->state.get_virt_heading();
    vec2f phys_pos = egocentric_user->state.get_phys_pos();
    vec2f virt_pos = egocentric_user->state.get_virt_pos();
    std::vector<trajectory_unit>* path = &(egocentric_user->state.path);
    std::deque<proximity_container*> prox_queue = egocentric_user->proximity_queue;
    physical_environment* phys_env = egocentric_user->physical_env();
    vec2f phys_env_center = egocentric_user->physical_env()->get_center();
    float user_radius = egocentric_user->radius;

    float gains;
    // Collided with a wall. Do regular reset to center
    if (closest_object->type == object::OBJECT_TYPE::WALL) {
        gains = reorient_to_target(phys_heading, virt_heading, phys_pos, virt_pos, phys_env_center, path);
    }
    // Collided with an obstacle
    else {
        // Obstacle is not between the user and the phys space center,
        // just reset to the center.
        if (!closest_object->is_blocking_path(phys_pos, phys_env_center, user_radius)) {
            gains = reorient_to_target(phys_heading, virt_heading, phys_pos, virt_pos, phys_env_center, path);
        }
        // Obstacle is between the user and the phys space center. Reset
        // parallel to side of the object that is closest to the center.
        else {
            vec2f wall_closest_to_center = closest_object->get_closest_wall(phys_pos);
            vec2f wall_normal = geom::normal_to_point(new vec2f(0.0f, 0.0f), &wall_closest_to_center, &phys_pos);
            wall_normal = wall_normal * 0.1f;
            vec2f p1 = phys_pos + wall_closest_to_center;
            vec2f p2 = phys_pos - wall_closest_to_center;
            if (length(p1 - phys_env_center) < length(p2 - phys_env_center)) {
                gains = reorient_to_target(phys_heading, virt_heading, phys_pos, virt_pos, phys_pos + wall_closest_to_center + wall_normal, path); // TODO: check if this actually works lol
            }
            else {
                gains = reorient_to_target(phys_heading, virt_heading, phys_pos, virt_pos, phys_pos - wall_closest_to_center + wall_normal, path); // TODO: check if this actually works lol
            }

        }
    }

    return gains;
}