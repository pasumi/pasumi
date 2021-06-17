#include <iostream>
#include <fstream>
#include <sstream>

#include "physical_environment.h"
#include "geometry.h"
#include "config.h"
#include "dynamic_obstacle.h"

physical_environment::physical_environment() {

}

physical_environment::physical_environment(std::vector<vec2f*> verts, std::vector<obstacle*> obstacles, vec2f center, char* name) {
	this->name = name;
	this->verts = verts;
	this->walls = std::vector<wall*>();
	this->obstacles = obstacles;
	num_vertices = verts.size();

	min_x = math::max_float;
	max_x = math::min_float;
	min_y = math::max_float;
	max_y = math::min_float;
	
	for (int i = 0; i < verts.size(); i++) {
		vec2f* p1 = verts[i % verts.size()];
		vec2f* p2 = verts[(i + 1) % verts.size()];
		walls.push_back(new wall(p1, p2, true, true));

		if (p1->x < min_x) min_x = p1->x;
		if (p1->x > max_x) max_x = p1->x;
		if (p1->y < min_y) min_y = p1->y;
		if (p1->y > max_y) max_y = p1->y;
	}

	this->center = center;
}

physical_environment::physical_environment(fs::path env_file) {
	this->load_xml_file(env_file);

	/*
	this->name = name;
	// Read file to create environment boundary
	std::fstream newfile;
	newfile.open(boundary_file, std::ios::in);

	if (newfile.is_open()) {
		std::string tp;
		while (getline(newfile, tp)) {
			std::stringstream ss(tp);
			int i = 0;
			vec2f* next_point = new vec2f(0.0f, 0.0f);
			while (ss.good()) {
				std::string substr;
				getline(ss, substr, ',');
				if (i % 2 == 0) next_point->x = std::stof(substr);
				else			next_point->y = std::stof(substr);
				i++;
			}
			verts.push_back(next_point);
		}
		newfile.close();
	}

	// Read file to create environment obstacles
	newfile.open(obstacles_file, std::ios::in);

	std::vector<obstacle*> obs;
	std::vector<vec2f*> cur_obs_verts;
	bool finished_obs = false;

	if (newfile.is_open()) {
		std::string tp;
		while (getline(newfile, tp)) {
			std::stringstream ss(tp);
			int i = 0;
			vec2f* next_point = new vec2f(0.0f, 0.0f);
			while (ss.good()) {
				std::string substr;
				getline(ss, substr, ',');

				if (substr[0] == '/') {
					obstacle* o = new obstacle(cur_obs_verts, true, true);
					obs.push_back(o);
					cur_obs_verts.clear();
					finished_obs = true;
					continue;
				}

				if (i % 2 == 0) next_point->x = std::stof(substr);
				else			next_point->y = std::stof(substr);
				i++;
			}
			if (!finished_obs) {
				cur_obs_verts.push_back(next_point);
			}
			else {
				finished_obs = false;
			}
		}
		newfile.close();
	}
	if (cur_obs_verts.size() > 0) {
		obstacle* o = new obstacle(cur_obs_verts, true, true);
		obs.push_back(o);
	}

	this->walls = std::vector<wall*>();
	this->obstacles = obs;
	num_vertices = verts.size();

	min_x = math::max_float;
	max_x = math::min_float;
	min_y = math::max_float;
	max_y = math::min_float;

	for (int i = 0; i < verts.size(); i++) {
		vec2f* p1 = verts[i % verts.size()];
		vec2f* p2 = verts[(i + 1) % verts.size()];
		walls.push_back(new wall(p1, p2, true, true));

		if (p1->x < min_x) min_x = p1->x;
		if (p1->x > max_x) max_x = p1->x;
		if (p1->y < min_y) min_y = p1->y;
		if (p1->y > max_y) max_y = p1->y;
	}

	this->center = vec2f(0.0f, 0.0f);
	*/
}

vec2f physical_environment::get_center() {
	return center;
}

vec2f physical_environment::sample_point() {
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

void physical_environment::step() {
	for (obstacle* o : obstacles) {
		o->step();
	}
}