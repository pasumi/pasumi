#include <iostream>
#include <fstream>
#include <sstream>

#include "dynamic_obstacle.h"

dynamic_obstacle::dynamic_obstacle() {

}

dynamic_obstacle::dynamic_obstacle(char* _path, bool is_phys) {
	path_index = 0;
	increment = 1;
	type = OBJECT_TYPE::OBSTACLE;
	space = (is_phys) ? SPACE_TYPE::PHYS : SPACE_TYPE::VIRT;
	mvmt = MOVEMENT_TYPE::DYNAMIC;

	// Read file to get object path
	std::fstream newfile;
	newfile.open(_path, std::ios::in);

	if (newfile.is_open()) {
		std::string tp;
		while (getline(newfile, tp)) {
			std::stringstream ss(tp);
			int i = 0;
			vec2f next_point = vec2f(0.0f, 0.0f);
			while (ss.good()) {
				std::string substr;
				getline(ss, substr, ',');

				if (i % 2 == 0) next_point.x = std::stof(substr);
				else			next_point.y = std::stof(substr);
				i++;
			}

			path.push_back(new vec2f(next_point.x, next_point.y));
		}
		newfile.close();
	}
}

dynamic_obstacle::~dynamic_obstacle() {
	for (auto v : path) {
		delete v;
	}
}

void dynamic_obstacle::update() {
	pos = path[path_index];
	path_index += increment;

	if (path_index == path.size()) {
		increment = -1;
	}
	if (path_index == 0) {
		increment = 1;
	}
}

std::vector<vec2f*> dynamic_obstacle::get_vertices() {
	std::vector<vec2f*> temp;
	vec2f* temp2 = new vec2f(pos->x, pos->y);
	temp.push_back(temp2);
	return temp;
}

/**
 * Distance from object to p.
 */
float dynamic_obstacle::distance(vec2f p) {
	return length(p - (*pos));
}

bool dynamic_obstacle::is_blocking_path(vec2f start, vec2f end, float radius) {
	vec2f path = end - start;
	vec2f perpendicular_offset = normalize(vec2f(-path.y, path.x)) * radius;
	float dist = geom::line_point_distance(new vec2f(start), new vec2f(end), pos);
	return (dist < radius);
}

/**
 * Here, the "walls" are a bounding box around the user.
 */
vec2f dynamic_obstacle::get_closest_wall(vec2f p) {
	std::vector<vec2f> bb;
	bb.push_back((*pos) + (radius * vec2f(1.0f, 0.0f)) + 
							(radius * vec2f(0.0f, 1.0f)));
	bb.push_back((*pos) - (radius * vec2f(1.0f, 0.0f)) + 
							(radius * vec2f(0.0f, 1.0f)));
	bb.push_back((*pos) + (radius * vec2f(1.0f, 0.0f)) - 
							(radius * vec2f(0.0f, 1.0f)));
	bb.push_back((*pos) - (radius * vec2f(1.0f, 0.0f)) - 
							(radius * vec2f(0.0f, 1.0f)));

	float best_dist = std::numeric_limits<float>::max();
	vec2f best_wall;
	for (int i = 0; i < bb.size(); i++) {
		float dist = geom::line_point_distance(new vec2f(bb[i]), new vec2f(bb[(i + 1) % bb.size()]), new vec2f(p));
		if (dist < best_dist) {
			best_dist = dist;
			best_wall = bb[(i + 1)] - bb[i];
		}
	}

	return normalize(best_wall);
}

bool dynamic_obstacle::is_dynamic() {
	return mvmt == MOVEMENT_TYPE::DYNAMIC;
}