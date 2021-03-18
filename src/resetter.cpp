#include "resetter.h"
#include "config.h"

resetter::resetter() {
	name = "base resetter";
}

resetter::~resetter() {

}

float resetter::reorient_to_target(vec2f phys_heading, float virt_heading, vec2f phys_pos, vec2f virt_pos, vec2f target, std::vector<trajectory_unit>* path) {
    float angle_per_dt = math::radians(config::ANGULAR_VELOCITY) / (1.0f / timestep::dt);

    float angle_to_target = signed_angle(phys_heading, normalize(target - phys_pos));
    float complement_angle = (math::pi * 2) - math::abs(angle_to_target); // Complement of angle to target
    complement_angle *= -math::sign(angle_to_target);	// Flip the direction

    /*
    redirection_unit resetting_gains;
    resetting_gains.apply_rota = true;
    resetting_gains.rota_gain = math::abs(complement_angle) / (math::pi * 2); // Gain needed to have the 360 virtual turn result in a physical turn towards the target (phys env center).
    resetting_gains.apply_trans = false;
    resetting_gains.trans_gain = 1.0f;
    resetting_gains.apply_curve = false;
    resetting_gains.curve_gain = 0.0f;
    resetting_gains.apply_bend = false;
    resetting_gains.bend_gain = 0.0f;
    */
    float rota_gain = math::abs(complement_angle) / (math::pi * 2); // Gain needed to have the 360 virtual turn result in a physical turn towards the target (phys env center).

    float cur_heading = virt_heading;

    int num_iter = math::two_pi / angle_per_dt;
    reset_timer = num_iter;
    path->insert(path->begin(), trajectory_unit(virt_pos.x, virt_pos.y, cur_heading));
    for (int i = 0; i < num_iter; i++) {
        if (i == 78) {
            int eriuth = 4;
        }
        cur_heading += angle_per_dt * math::sign(complement_angle) * -1;
        cur_heading = math::clamp_rotation(cur_heading);
        path->insert(path->begin(), trajectory_unit(virt_pos.x, virt_pos.y, cur_heading));
    }
    float remainder = math::two_pi - (angle_per_dt * num_iter);
    if (remainder) {
        cur_heading += remainder;
        cur_heading = math::clamp_rotation(cur_heading);
        path->insert(path->begin(), trajectory_unit(virt_pos.x, virt_pos.y, cur_heading));
        reset_timer++;
    }

    return rota_gain;
}