#include "no_rdw.h"
#include "config.h"

no_rdw::no_rdw(resetter* _resetter) {
    name = "No redirection";
	steer_target = vec2f(0.0f, 0.0f);
	gradient_dir = vec2f(0.0f, 0.0f);
    for (int i = 0; i < GRADIENT_SAMPLE_RATE; i++) {
        theta_values.push_back(((2 * math::pi) / GRADIENT_SAMPLE_RATE) * i);
    }

	cur_alignment = std::vector<float>{ 0.0f, 0.0f, 0.0f };

	reset_policy = _resetter;
}

redirection_unit no_rdw::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
    //visibility_polygon cur_phys_vis_poly = get_current_visibility_polygon(egocentric_user->state.get_phys_pos(), (environment*)(egocentric_user->physical_env()));
    //visibility_polygon cur_virt_vis_poly = get_current_visibility_polygon(egocentric_user->state.get_virt_pos(), sim_state.virt_env);

    //update_losses(cur_phys_vis_poly, cur_virt_vis_poly, sim_state, egocentric_user);
    cur_alignment = std::vector<float>{ cur_north_loss, cur_east_loss, cur_west_loss };

	redirection_unit redir_unit = redirection_unit();
	return redir_unit;
}

visibility_polygon no_rdw::get_current_visibility_polygon(vec2f pos, environment* env) {
	// https://github.com/trylock/visibility
	using vector_type = geometry::vec2;
	using segment_type = geometry::line_segment<vector_type>;
	using segment_comparer_type = geometry::line_segment_dist_comparer<vector_type>;
	using angle_comparer_type = geometry::angle_comparer<vector_type>;
	using namespace geometry;

	std::vector<segment_type> segments;
	for (int i = 0; i < env->get_vertices().size(); i++) {
		vec2f* p1 = env->get_vertices()[i];
		vec2f* p2 = env->get_vertices()[(i + 1) % env->get_vertices().size()];
		segments.push_back({ {p1->x, p1->y}, {p2->x, p2->y} });
	}
	for (obstacle* o : env->get_obstacles()) {
		for (int i = 0; i < o->get_vertices().size(); i++) {
			vec2f* p1 = o->get_vertices()[i];
			vec2f* p2 = o->get_vertices()[(i + 1) % o->get_vertices().size()];
			segments.push_back({ {p1->x, p1->y}, {p2->x, p2->y} });
		}
	}

	//auto poly = visibility_polygon2(vector_type{ pos.x, pos.y }, segments.begin(), segments.end());
	//std::vector<vec2f> temp;
	//for (vec2 p : poly) {
		//temp.insert(temp.begin(), vec2f(p.x, p.y)); // Want it in CCW order
	//}
	//return visibility_polygon(temp, pos);

	return visibility_polygon(env, &pos);
}

void no_rdw::update_losses(visibility_polygon& phys_poly, visibility_polygon& virt_poly, simulation_state& sim_state, user* egocentric_user) {
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