#include <vector>
#include <iostream>

#include "wall.h"
#include "geometry.h"

wall::wall() {

}

wall::wall(vec2f* p1, vec2f* p2, bool is_phys, bool is_static) {
	this->p1 = new vec2f(p1->x, p1->y);
	this->p2 = new vec2f(p2->x, p2->y);
	//this->verts = std::vector<vec2f> { p1, p2 };
	verts.push_back(this->p1);
	verts.push_back(this->p2);
	num_vertices = 2;

	type = object::OBJECT_TYPE::WALL;
	space = (is_phys) ? object::SPACE_TYPE::PHYS : object::SPACE_TYPE::VIRT;
	mvmt = (is_static) ? object::MOVEMENT_TYPE::STATIC : object::MOVEMENT_TYPE::DYNAMIC;
}

wall::~wall() {
	//delete p1;
	//delete p2;
	for (vec2f* v : verts) {
		delete v;
	}
}

//wall::wall(const wall& w) {
//	this->p1 = w.p1;
//	this->p2 = w.p2;
//	this->verts = std::vector<vec2f>{ p1, p2 };
//	num_vertices = 2;
//
//	type = w.type;
//	space = w.space;
//	mvmt = w.mvmt;
//}

int wall::vertex_buffer_size() {
	return num_vertices * 3 * sizeof(float);
}

int wall::index_buffer_size() {
	return num_vertices * sizeof(int);
}

std::vector<vec2f*> wall::get_vertices() {
	return verts;
}

float wall::distance(vec2f p) {
	return geom::line_point_distance(p1, p2, &p);
}

bool wall::is_blocking_path(vec2f start, vec2f end, float radius) {
	vec2f path = end - start;
	vec2f perpendicular_offset = normalize(vec2f(-path.y, path.x)) * radius;

	vec2f path1_p1 = start + perpendicular_offset;
	vec2f path1_p2 = end + perpendicular_offset;
	vec2f path2_p1 = start - perpendicular_offset;
	vec2f path2_p2 = end - perpendicular_offset;

	return geom::line_line_intersect(p1, p2, new vec2f(path1_p1), new vec2f(path1_p2));
}

vec2f wall::get_closest_wall(vec2f p) {
	return normalize(*p2 - *p1);
}

bool wall::is_dynamic() {
	return mvmt == MOVEMENT_TYPE::DYNAMIC;
}