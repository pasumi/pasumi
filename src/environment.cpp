#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <time.h>
#include <iomanip>
#include <string>

#ifdef _WIN32
#include <direct.h>
#endif

#ifdef linux
#endif

#include "environment.h"
#include "geometry.h"
#include "config.h"
#include "ext/pugixml/pugixml.hpp"
#include "ext/ghc/filesystem.hpp"

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

void environment::load_xml_file(fs::path filepath) {
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(filepath.string().c_str());

	this->name = (char*)doc.child("env").attribute("name").as_string();
	bool is_phys = ((char*)doc.child("env").attribute("type").as_string()) == "physical";

	// Boundary
	parse_border(doc.child("env").child("border"));

	// Obstacles
	for (pugi::xml_node obs = doc.child("env").child("obstacle"); obs; obs = obs.next_sibling("obstacle")) {
		obstacle* o = parse_obstacle(obs, is_phys);
		obstacles.push_back(o);
	}
}

void environment::parse_border(pugi::xml_node border_node) {
	// Get the vertex data from the XML file
	for (pugi::xml_node vert = border_node.child("vertices").child("vertex"); vert; vert = vert.next_sibling("vertex")) {
		vec2f v = parse_vertex((char*)vert.child_value());
		verts.push_back(new vec2f(v.x, v.y));
	}

	// Create wall objects
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
	this->center = parse_vertex((char*)border_node.child("vertex").child_value());
}

vec2f environment::parse_vertex(char* vert_string) {
	std::string vert_data = (std::string)vert_string;
	std::string delimiter = ",";
	float x = std::stof(vert_data.substr(0, vert_data.find(delimiter)));
	float y = std::stof(vert_data.substr(vert_data.find(delimiter)+1));
	return vec2f(x, y);
}

obstacle* environment::parse_obstacle(pugi::xml_node obs_node, bool is_phys) {
	std::vector<vec2f*> obs_verts;
	bool is_static = ((char*)obs_node.attribute("type").as_string()) == "static";

	for (pugi::xml_node vert = obs_node.child("vertices").child("vertex"); vert; vert = vert.next_sibling("vertex")) {
		vec2f v = parse_vertex((char*)vert.child_value());
		obs_verts.push_back(new vec2f(v.x, v.y));
	}

	return new obstacle(obs_verts, is_phys, is_static);
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