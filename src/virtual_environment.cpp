#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>

#include "virtual_environment.h"
#include "geometry.h"
#include "config.h"

virtual_environment::virtual_environment() {

}

virtual_environment::virtual_environment(std::vector<vec2f*> verts, char* name) {
	this->name = name;
	this->verts = verts;
	this->walls = std::vector<wall*>();
	unbounded = verts.size() == 0;

	if (unbounded) {
		min_x = math::min_float;
		max_x = math::max_float;
		min_y = math::min_float;
		max_y = math::max_float;
	}
	else {
		min_x = math::max_float;
		max_x = math::min_float;
		min_y = math::max_float;
		max_y = math::min_float;
		for (int i = 0; i < verts.size(); i++) {
			vec2f* p1 = verts[i % verts.size()];
			vec2f* p2 = verts[(i + 1) % verts.size()];
			walls.push_back(new wall(p1, p2, false, true));

			if (p1->x < min_x) min_x = p1->x;
			if (p1->x > max_x) max_x = p1->x;
			if (p1->y < min_y) min_y = p1->y;
			if (p1->y > max_y) max_y = p1->y;
		}
	}
}

virtual_environment::virtual_environment(fs::path env_file) {
	this->load_xml_file(env_file);
}

vec2f virtual_environment::sample_point() {
	float rand_x = min_x + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max_x - min_x)));
	float rand_y = min_y + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max_y - min_y)));
	vec2f sampled_point = vec2f(rand_x, rand_y);

	bool is_far_enough_away = get_closest_obstacle_distance(sampled_point) > config::RESET_DISTANCE_CHECK_VALUE;
	while (!point_is_legal(sampled_point) || !is_far_enough_away) {
		rand_x = min_x + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max_x - min_x)));
		rand_y = min_y + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max_y - min_y)));
		sampled_point = vec2f(rand_x, rand_y);
		is_far_enough_away = get_closest_obstacle_distance(sampled_point) > config::RESET_DISTANCE_CHECK_VALUE;
	}

	assert(point_is_legal(sampled_point) && "Sampled virtual point is not legal");
	return sampled_point;
}

void virtual_environment::step() {
	for (obstacle* o : obstacles) {
		o->step();
	}
}