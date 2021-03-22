#include <iostream>
#include <fstream>
#include <sstream>

#include "physical_environment.hpp"
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

		gl_wall_verts.push_back(verts[i]->x);
		gl_wall_verts.push_back(verts[i]->y);
		gl_wall_verts.push_back(0.0f);

		gl_wall_indices.push_back(i);
		gl_wall_indices.push_back((i + 1) % verts.size());

		if (p1->x < min_x) min_x = p1->x;
		if (p1->x > max_x) max_x = p1->x;
		if (p1->y < min_y) min_y = p1->y;
		if (p1->y > max_y) max_y = p1->y;
	}

	int obstacle_verts_offset = 0;
	for (int i = 0; i < obstacles.size(); i++) {
		std::vector<vec2f*> obs_verts = obstacles[i]->get_vertices();
		for (int j = 0; j < obs_verts.size(); j++) {
			gl_obstacle_verts.push_back(obs_verts[j]->x);
			gl_obstacle_verts.push_back(obs_verts[j]->y);
			gl_obstacle_verts.push_back(0.0f);

			gl_obstacle_indices.push_back(obstacle_verts_offset + j);
			gl_obstacle_indices.push_back((unsigned int)(obstacle_verts_offset + ((j + 1) % obs_verts.size())));
		}
		obstacle_verts_offset += obs_verts.size();
	}

	this->center = center;
	//compute_features(name);
}

physical_environment::physical_environment(std::string boundary_file, std::string obstacles_file, char* name) {
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
	//obs.push_back(new obstacle("C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/virt/obstacles/dynamic_path1.txt", true, false, 0.5f));
	//obs.push_back(new obstacle("C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/virt/obstacles/dynamic_path2.txt", true, false, 0.5f));

	//this->verts = verts;
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

		gl_wall_verts.push_back(verts[i]->x);
		gl_wall_verts.push_back(verts[i]->y);
		gl_wall_verts.push_back(0.0f);

		gl_wall_indices.push_back(i);
		gl_wall_indices.push_back((i + 1) % verts.size());

		if (p1->x < min_x) min_x = p1->x;
		if (p1->x > max_x) max_x = p1->x;
		if (p1->y < min_y) min_y = p1->y;
		if (p1->y > max_y) max_y = p1->y;
	}

	int obstacle_verts_offset = 0;
	for (int i = 0; i < obstacles.size(); i++) {
		std::vector<vec2f*> obs_verts = obstacles[i]->get_vertices();
		for (int j = 0; j < obs_verts.size(); j++) {
			gl_obstacle_verts.push_back(obs_verts[j]->x);
			gl_obstacle_verts.push_back(obs_verts[j]->y);
			gl_obstacle_verts.push_back(0.0f);

			gl_obstacle_indices.push_back(obstacle_verts_offset + j);
			gl_obstacle_indices.push_back((unsigned int)(obstacle_verts_offset + ((j + 1) % obs_verts.size())));
		}
		obstacle_verts_offset += obs_verts.size();
	}

	this->center = vec2f(0.0f, 0.0f);
}

int physical_environment::vertex_buffer_size() {
	return gl_wall_verts.size() * sizeof(float);
}

int physical_environment::index_buffer_size() {
	return gl_wall_indices.size() * sizeof(unsigned int);
}

int physical_environment::wall_vertex_buffer_size() {
	return gl_wall_verts.size() * sizeof(float);
}

int physical_environment::wall_index_buffer_size() {
	return gl_wall_indices.size() * sizeof(unsigned int);
}

int physical_environment::obstacle_vertex_buffer_size() {
	return gl_obstacle_verts.size() * sizeof(float);
}

int physical_environment::obstacle_index_buffer_size() {
	return gl_obstacle_indices.size() * sizeof(unsigned int);
}

/*
std::vector<vec2f> physical_environment::get_vertices() {
	return verts;
}

std::vector<wall*> physical_environment::get_walls() {
	return walls;
}

std::vector<obstacle*> physical_environment::get_obstacles() {
	return obstacles;
}

bool physical_environment::point_is_legal(vec2f p) {
	if (!geom::point_in_polygon(p, verts)) return false;

	for (obstacle* obstacle : obstacles) {
		if (geom::point_in_polygon(p, obstacle->get_vertices())) return false;
	}

	return true;
}

bool physical_environment::line_is_legal(vec2f* p1, vec2f* p2) {
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

	int obstacle_verts_offset = 0;
	gl_obstacle_verts.clear();
	gl_obstacle_indices.clear();
	for (int i = 0; i < obstacles.size(); i++) {
		std::vector<vec2f*> obs_verts = obstacles[i]->get_vertices();
		for (int j = 0; j < obs_verts.size(); j++) {
			gl_obstacle_verts.push_back(obs_verts[j]->x);
			gl_obstacle_verts.push_back(obs_verts[j]->y);
			gl_obstacle_verts.push_back(0.0f);

			gl_obstacle_indices.push_back(obstacle_verts_offset + j);
			gl_obstacle_indices.push_back((unsigned int)(obstacle_verts_offset + ((j + 1) % obs_verts.size())));
		}
		obstacle_verts_offset += obs_verts.size();
	}
}