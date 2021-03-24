#include "reset_to_forward_distance.h"

reset_to_forward_distance::reset_to_forward_distance() {
	name = "Reset to forward distance";
	for (int i = 0; i < SAMPLE_RATE; i++) {
		sample_directions.push_back(((2 * math::pi) / SAMPLE_RATE) * i);
	}
}

reset_to_forward_distance::~reset_to_forward_distance() {

}

float reset_to_forward_distance::reset(simulation_state& sim_state, user* egocentric_user) {
	proximity_container* closest_obj_container = egocentric_user->get_closest_obstacle(object::SPACE_TYPE::PHYS);
	std::vector<vec2f*> closest_obj = closest_obj_container->get_closest_feature();
	vec2f cur_phys_pos = egocentric_user->state.get_phys_pos();

	vec2f closest_wall_normal;
	if (closest_obj.size() > 1) {
		closest_wall_normal = geom::normal_to_point(closest_obj[0], closest_obj[1], &cur_phys_pos);
	}
	else {
		closest_wall_normal = rad_2_vec(((user*)(closest_obj_container->get_obstacle()))->state.get_phys_heading());
	}

	vec2f virt_pos = egocentric_user->state.get_virt_pos();
	vec2f virt_heading = rad_2_vec(egocentric_user->state.get_virt_heading());
	float virt_dist_north = egocentric_user->virtual_env()->get_distance_in_direction_from_point(virt_pos, virt_heading);

	float best_loss = math::max_float;
	vec2f best_dir;
	float best_under_loss = math::max_float;
	vec2f best_under_dir;
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	for (float d : sample_directions) {
		vec2f dir = rad_2_vec(d);
		// Ensure we face away from the wall after resetting.
		if (dot(dir, closest_wall_normal) < math::epsilon) continue;

		vec2f phys_heading = rad_2_vec(egocentric_user->state.get_phys_heading());
		float phys_dist_north = egocentric_user->physical_env()->get_distance_in_direction_from_point(phys_pos, dir);

		float loss_north = phys_dist_north - virt_dist_north;

		float sum = math::abs(loss_north);
		if (sum < best_loss) {
			best_loss = sum;
			best_dir = dir;
		}

		if (phys_dist_north < virt_dist_north) {
			float sum = math::abs(loss_north);
			if (sum < best_under_loss) {
				best_under_loss = sum;
				best_under_dir = dir;
			}
		}
		else {
			float sum = math::abs(loss_north);
			if (sum < best_loss) {
				best_loss = sum;
				best_dir = dir;
			}
		}
	}


	if (best_loss == math::max_float) {
		best_dir = best_under_dir;
	}
	return reorient_to_target(
		rad_2_vec(egocentric_user->state.get_phys_heading()),
		egocentric_user->state.get_virt_heading(), 
		cur_phys_pos, 
		egocentric_user->state.get_virt_pos(), 
		cur_phys_pos + best_dir, 
		&(egocentric_user->state.path)
	);
}