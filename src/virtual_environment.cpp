#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>

#include "virtual_environment.hpp"
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
		min_x = std::numeric_limits<float>::min();
		max_x = std::numeric_limits<float>::max();
		min_y = std::numeric_limits<float>::min();
		max_y = std::numeric_limits<float>::max();
	}
	else {
		min_x = std::numeric_limits<float>::max();
		max_x = std::numeric_limits<float>::min();
		min_y = std::numeric_limits<float>::max();
		max_y = std::numeric_limits<float>::min();
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
	build_cgal_data();
	//compute_features(name);
}

virtual_environment::virtual_environment(std::string boundary_file, std::string obstacles_file, char* name) {
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
	min_x = std::numeric_limits<float>::max();
	max_x = std::numeric_limits<float>::min();
	min_y = std::numeric_limits<float>::max();
	max_y = std::numeric_limits<float>::min();
	for (int i = 0; i < verts.size(); i++) {
		vec2f* p1 = verts[i % verts.size()];
		vec2f* p2 = verts[(i + 1) % verts.size()];
		walls.push_back(new wall(p1, p2, false, true));

		if (p1->x < min_x) min_x = p1->x;
		if (p1->x > max_x) max_x = p1->x;
		if (p1->y < min_y) min_y = p1->y;
		if (p1->y > max_y) max_y = p1->y;
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
			vec2f next_point = vec2f(0.0f, 0.0f);
			while (ss.good()) {
				std::string substr;
				getline(ss, substr, ',');

				if (substr[0] == '/') {
					obstacle* o = new obstacle(cur_obs_verts, false, true);
					obs.push_back(o);
					cur_obs_verts.clear();
					finished_obs = true;
					continue;
				}

				if (i % 2 == 0) next_point.x = std::stof(substr);
				else			next_point.y = std::stof(substr);
				i++;
			}
			if (!finished_obs) {
				cur_obs_verts.push_back(new vec2f(next_point.x, next_point.y));
			}
			else {
				finished_obs = false;
			}
		}
		newfile.close();
	}
	if (cur_obs_verts.size() > 0) {
		obstacle* o = new obstacle(cur_obs_verts, false, true);
		obs.push_back(o);
	}

	for (vec2f* v : cur_obs_verts) {
		delete v;
	}
	this->obstacles = obs;
	//setup_RVO_obstacles();
	build_cgal_data();
	//compute_features(name);
}

void virtual_environment::setup_RVO_obstacles() {
	std::vector<obstacle*> temp;
	for (int i = 1; i < 5; i++) {
		obstacle* o = new obstacle(false, false, 0.5f);
		temp.push_back(o);
		//o->load_RVO_data(i);
		vec2f new_pos = this->sample_point();
		o->set_pos(new_pos);
	}
	for (auto o : temp) {
		this->obstacles.push_back(o);
	}
}

/*
std::vector<vec2f> virtual_environment::get_vertices() {
	return verts;
}

std::vector<wall*> virtual_environment::get_walls() {
	return walls;
}

std::vector<obstacle*> virtual_environment::get_obstacles() {
	return obstacles;
}

bool virtual_environment::point_is_legal(vec2f p) {
	if (unbounded) return true;

	if (!geom::point_in_polygon(p, verts)) return false;

	for (obstacle* obstacle : obstacles) {
		if (!geom::point_in_polygon(p, obstacle->get_vertices())) return false;
	}

	return true;
}

bool virtual_environment::line_is_legal(vec2f* p1, vec2f* p2) {
	for (wall* w : walls) {
		vec2f wall_p1 = w->get_vertices()[0];
		vec2f wall_p2 = w->get_vertices()[1];
		if (geom::line_line_intersect(*p1, *p2, wall_p1, wall_p2)) return false;
	}

	for (obstacle* o : obstacles) {
		for (wall w : o->get_walls()) {
			vec2f wall_p1 = w.get_vertices()[0];
			vec2f wall_p2 = w.get_vertices()[1];
			if (geom::line_line_intersect(*p1, *p2, wall_p1, wall_p2)) return false;
		}
	}

	return true;
}
*/

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