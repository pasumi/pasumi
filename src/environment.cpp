#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <time.h>
#include <iomanip>

#ifdef _WIN32
#include <direct.h>
#endif

#ifdef linux
#endif

#include "environment.h"
#include "geometry.h"
#include "config.h"

environment::environment() {

}

environment::~environment() {
	for (auto v : verts) {
		delete v;
	}

	for (auto w : walls) {
		delete w;
	}

	for (auto o : obstacles) {
		delete o;
	}
}

std::vector<vec2f*> environment::get_vertices() {
	return verts;
}

std::vector<wall*> environment::get_walls() {
	return walls;
}

std::vector<obstacle*> environment::get_obstacles() {
	return obstacles;
}

bool environment::point_is_legal(vec2f p) {
	if (unbounded) return true;

	if (!geom::point_in_polygon(p, verts)) return false;

	for (obstacle* obstacle : obstacles) {
		if (geom::point_in_polygon(p, obstacle->get_vertices())) return false;
	}

	return true;
}

bool environment::line_is_legal(vec2f* p1, vec2f* p2) {
	for (wall* w : walls) {
		vec2f* wall_p1 = w->get_vertices()[0];
		vec2f* wall_p2 = w->get_vertices()[1];
		if (geom::line_line_intersect(p1, p2, wall_p1, wall_p2)) return false;
	}

	for (obstacle* o : obstacles) {
		for (wall* w : o->get_walls()) {
			vec2f* wall_p1 = w->get_vertices()[0];
			vec2f* wall_p2 = w->get_vertices()[1];
			if (geom::line_line_intersect(p1, p2, wall_p1, wall_p2)) return false;
		}
	}

	return true;
}

float environment::get_closest_obstacle_distance(vec2f p) {
	float closest = math::max_float;
	for (wall* w : walls) {
		vec2f* wall_p1 = w->get_vertices()[0];
		vec2f* wall_p2 = w->get_vertices()[1];
		float d = geom::line_point_distance(wall_p1, wall_p2, &p);
		if (d < closest) closest = d;
	}

	for (obstacle* o : obstacles) {
		for (wall* w : o->get_walls()) {
			vec2f* wall_p1 = w->get_vertices()[0];
			vec2f* wall_p2 = w->get_vertices()[1];
			float d = geom::line_point_distance(wall_p1, wall_p2, &p);
			if (d < closest) closest = d;
		}
	}

	return closest;
}

float environment::get_distance_in_direction_from_point(vec2f p, vec2f dir) {
	float closest = math::max_float;
	vec2f* p1;
	vec2f* p2;

	for (wall* w : walls) {
		p1 = w->get_vertices()[0];
		p2 = w->get_vertices()[1];
		float t = geom::ray_line_intersect(&p, &dir, p1, p2);
		if (t >= 0 && t < closest) {
			closest = t;
		}
	}
	for (obstacle* o : obstacles) {
		for (wall* w : o->get_walls()) {
			p1 = w->get_vertices()[0];
			p2 = w->get_vertices()[1];
			float t = geom::ray_line_intersect(&p, &dir, p1, p2);
			if (t >= 0 && t < closest) {
				closest = t;
			}
		}
	}

	return closest;
}

void environment::step() {
	for (obstacle* o : obstacles) {
		o->step();
	}
}