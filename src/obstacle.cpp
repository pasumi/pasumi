#pragma once

#include <climits>

#include "obstacle.h"
#include "geometry.h"

obstacle::obstacle() {

}

obstacle::obstacle(std::vector<vec2f*> _verts, bool is_phys, bool is_static) {
	for (vec2f* v : _verts) {
		this->verts.push_back(new vec2f(v->x, v->y));
	}
	this->walls = std::vector<wall*>();
	num_vertices = verts.size();
	type = object::OBJECT_TYPE::OBSTACLE;
	space = (is_phys) ? object::SPACE_TYPE::PHYS : object::SPACE_TYPE::VIRT;
	mvmt = (is_static) ? object::MOVEMENT_TYPE::STATIC : object::MOVEMENT_TYPE::DYNAMIC;
	cur_path = 0;

	for (int i = 0; i < verts.size(); i++) {
		vec2f* p1 = verts[i % verts.size()];
		vec2f* p2 = verts[(i + 1) % verts.size()];
		wall* w = new wall(p1, p2, is_phys, is_static);
		walls.push_back(w);

		gl_verts.push_back(verts[i]->x);
		gl_verts.push_back(verts[i]->y);
		gl_verts.push_back(0.0f);

		gl_indices.push_back(i);
		gl_indices.push_back((i + 1) % verts.size());
	}
}

obstacle::obstacle(char* _path, bool is_phys, bool is_static, float _radius) {
	assert(!is_static, "Called dynamic obstacle constructor on a static object!");

	path_index = 0;
	increment = 1;
	type = OBJECT_TYPE::OBSTACLE;
	space = (is_phys) ? SPACE_TYPE::PHYS : SPACE_TYPE::VIRT;
	mvmt = MOVEMENT_TYPE::DYNAMIC;
	radius = _radius;
	cur_path = 0;

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
	pos = path[path_index];

	int num_circle_verts = 20;
	for (int i = 0; i < num_circle_verts; i++) {
		float theta = math::two_pi * ((float)i / (float)num_circle_verts);
		float x = math::cos(theta) * radius;
		float y = math::sin(theta) * radius;
		this->base_verts.push_back(new vec2f(x, y));
		this->verts.push_back(new vec2f(x + pos->x, y + pos->y));
	}
	for (int i = 0; i < verts.size(); i++) {
		vec2f* p1 = verts[i % verts.size()];
		vec2f* p2 = verts[(i + 1) % verts.size()];
		wall* w = new wall(p1, p2, is_phys, is_static);
		walls.push_back(w);

		gl_verts.push_back(verts[i]->x);
		gl_verts.push_back(verts[i]->y);
		gl_verts.push_back(0.0f);

		gl_indices.push_back(i);
		gl_indices.push_back((i + 1) % verts.size());
	}
}

obstacle::obstacle(bool is_phys, bool is_static, float _radius) {
	assert(!is_static, "Called dynamic obstacle constructor on a static object!");

	path_index = 0;
	increment = 1;
	type = OBJECT_TYPE::OBSTACLE;
	space = (is_phys) ? SPACE_TYPE::PHYS : SPACE_TYPE::VIRT;
	mvmt = MOVEMENT_TYPE::DYNAMIC;
	radius = _radius;
	cur_path = 0;
}

obstacle::~obstacle() {
	for (auto w : walls) {
		delete w;
	}
	for (auto v : this->verts) {
		delete v;
	}
}

void obstacle::set_pos(vec2f p) {
	pos = new vec2f(p.x, p.y);
}

int obstacle::vertex_buffer_size() {
	return gl_verts.size() * sizeof(float);
}

int obstacle::index_buffer_size() {
	return gl_indices.size() * sizeof(unsigned int);
}

std::vector<vec2f*> obstacle::get_vertices() {
	return verts;
}

std::vector<wall*> obstacle::get_walls() {
	return walls;
}

float obstacle::distance(vec2f p) {
	// Dynamic obstacles are currently limited to circles
	if (mvmt == MOVEMENT_TYPE::DYNAMIC) {
		return length(p - (*pos));
	}

	// Static polygonal obstacles
	float best_dist = std::numeric_limits<float>::max();
	for (wall* w : walls) {
		float dist = w->distance(p);
		best_dist = (dist < best_dist) ? dist : best_dist;
	}
	return best_dist;
}

bool obstacle::is_blocking_path(vec2f start, vec2f end, float radius) {
	// Dynamic obstacles are currently limited to circles
	if (mvmt == MOVEMENT_TYPE::DYNAMIC) {
		vec2f path = end - start;
		vec2f perpendicular_offset = normalize(vec2f(-path.y, path.x)) * radius;
		float dist = geom::line_point_distance(new vec2f(start), new vec2f(end), pos);
		return (dist < radius);
	}

	vec2f path = end - start;
	vec2f perpendicular_offset = normalize(vec2f(-path.y, path.x)) * radius;

	vec2f* path1_p1 = new vec2f(start + perpendicular_offset);
	vec2f* path1_p2 = new vec2f(end + perpendicular_offset);
	vec2f* path2_p1 = new vec2f(start - perpendicular_offset);
	vec2f* path2_p2 = new vec2f(end - perpendicular_offset);

	for (wall* w : walls) {
		vec2f* wall_p1 = w->get_vertices()[0];
		vec2f* wall_p2 = w->get_vertices()[1];
		if (geom::line_line_intersect(wall_p1, wall_p2, path1_p1, path1_p2) ||
			geom::line_line_intersect(wall_p1, wall_p2, path2_p1, path2_p2)) return true;
	}

	return false;
}

vec2f obstacle::get_closest_wall(vec2f p) {
	float closest_dist = std::numeric_limits<float>::max();
	vec2f closest_wall;

	for (wall* w: walls) {
		vec2f* wall_p1 = w->get_vertices()[0];
		vec2f* wall_p2 = w->get_vertices()[1];
		float dist = geom::line_point_distance(wall_p1, wall_p2, new vec2f(p));
		if (dist < closest_dist) {
			closest_dist = dist;
			closest_wall = *wall_p2 - *wall_p1;
		}
	}

	return normalize(closest_wall);
}

void obstacle::step() {
	if (mvmt == MOVEMENT_TYPE::STATIC) {
		return;
	}
	if (post_reset_timer > 0) {
		post_reset_timer--;
		return;
	}

	pos = path[path_index];
	path_index += increment;

	if (path_index == path.size() - 1) {
		increment = 0;
	}
	if (path_index == 0) {
		increment = 1;
	}

	// Update vertices of dynamic obstacle
	verts.clear();
	gl_verts.clear();
	for (wall* w : walls) {
		delete w;
	}
	walls.clear();
	for (vec2f* v : base_verts) {
		verts.push_back(new vec2f(v->x + pos->x, v->y + pos->y));
	}
	for (int i = 0; i < verts.size(); i++) {
		vec2f* p1 = verts[i % verts.size()];
		vec2f* p2 = verts[(i + 1) % verts.size()];
		wall* w = new wall(p1, p2, space==SPACE_TYPE::PHYS, false);
		walls.push_back(w);

		gl_verts.push_back(verts[i]->x);
		gl_verts.push_back(verts[i]->y);
		gl_verts.push_back(0.0f);

		gl_indices.push_back(i);
		gl_indices.push_back((i + 1) % verts.size());
	}
}

void obstacle::wait_for_reset(int wait_timer) {
	if (space == SPACE_TYPE::PHYS) {
		post_reset_timer = RESET_WAIT_LIMIT + wait_timer;
	}
	else {
		post_reset_timer = wait_timer + 1;
	}
}

bool obstacle::is_dynamic() {
	return mvmt == MOVEMENT_TYPE::DYNAMIC;
}

void obstacle::reset_state() {
	path_index = 0;
	if (rvo_paths.size() > 0) {
		cur_path++;
		path = rvo_paths[cur_path];
	}	
	pos = path[path_index];
	increment = 1;

	verts.clear();
	gl_verts.clear();
	for (wall* w : walls) {
		delete w;
	}
	walls.clear();
	for (vec2f* v : base_verts) {
		verts.push_back(new vec2f(v->x + pos->x, v->y + pos->y));
	}
	for (int i = 0; i < verts.size(); i++) {
		vec2f* p1 = verts[i % verts.size()];
		vec2f* p2 = verts[(i + 1) % verts.size()];
		wall* w = new wall(p1, p2, space == SPACE_TYPE::PHYS, false);
		walls.push_back(w);

		gl_verts.push_back(verts[i]->x);
		gl_verts.push_back(verts[i]->y);
		gl_verts.push_back(0.0f);

		gl_indices.push_back(i);
		gl_indices.push_back((i + 1) % verts.size());
	}
}

void obstacle::add_path(std::vector<vec2f*> new_path) {
	path.clear();
	path = new_path;
}