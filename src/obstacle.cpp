#pragma once

#include <climits>

#include "obstacle.h"
#include "geometry.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Triangulation_conformer_2.h>

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
	build_cgal_data();
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

void obstacle::load_RVO_data(int agent_id) {
	char* path_dir = "C:/Users/Niall Williams/Dropbox/UMD/Research/RDW Steering/simulated-rdw/envs/paths/";
	for (int i = 1; i <= 100; i++) {
		char filename[300];
		char temp[50];
		strcpy(filename, path_dir);
		sprintf(temp, "agent_%d_path_#%d_movement_history.csv", agent_id, i);
		strcat(filename, temp);

		std::fstream newfile;
		newfile.open(filename, std::ios::in);
		std::vector<vec2f> pts;

		bool first_line = true;
		if (newfile.is_open()) {
			std::string tp;
			while (getline(newfile, tp)) {
				if (first_line) {
					first_line = false;
					continue;
				}
				std::stringstream ss(tp);
				int i = 0;
				vec2f next_point;
				while (ss.good()) {
					std::string substr;
					getline(ss, substr, ',');
					if (i % 2 == 0) next_point.x = std::stof(substr);
					else			next_point.y = std::stof(substr);
					i++;
				}
				//new_path.waypoints.push_back(next_point);
				pts.push_back(next_point);
			}
			newfile.close();
		}

		std::vector<vec2f*> new_path;
		// Create the path from the points we just read in
		for (int j = 0; j < pts.size(); j++) {
			vec2f p1 = pts[j];
			vec2f p2 = pts[(j + 1) % pts.size()];
			float heading = vec_2_rad(normalize(p2 - p1));
			int t = 4;
			if ((j + 1) % pts.size() != 0) {
				new_path.push_back(new vec2f(p1.x, p1.y));
			}
			else {
				new_path.push_back(new vec2f(p1.x, p1.y));
			}
		}
		rvo_paths.push_back(new_path);
	}

	int num_circle_verts = 8;
	vec2f* pos = rvo_paths[0][0];
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
		wall* w = new wall(p1, p2, space == SPACE_TYPE::PHYS, mvmt == MOVEMENT_TYPE::DYNAMIC);
		walls.push_back(w);

		gl_verts.push_back(verts[i]->x);
		gl_verts.push_back(verts[i]->y);
		gl_verts.push_back(0.0f);

		gl_indices.push_back(i);
		gl_indices.push_back((i + 1) % verts.size());
	}
	cur_path = 0;
	path = rvo_paths[cur_path];
	std::cout << "Finished reading RVO data paths for obstacle.\n";
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

void obstacle::build_cgal_data() {
	// Environment boundary
	for (int i = 0; i < verts.size(); i++) {
		Point_2 p1(verts[i]->x, verts[i]->y);
		Point_2 p2(verts[(i + 1) % verts.size()]->x,
			verts[(i + 1) % verts.size()]->y);
		cgal_verts.push_back(Segment_2(p1, p2));
	}
}

vec2f obstacle::sample_point_inside() {
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef CGAL::Triangulation_vertex_base_2<K> Vb;
	typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
	typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
	typedef CDT::Vertex_handle Vertex_handle;
	typedef CDT::Point Point;
	typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;

	CDT cdt;
	std::vector<Point> pts;
	std::vector<Vertex_handle> handles;

	for (int i = 0; i < cgal_verts.size(); i++) {
		Segment_2 s = cgal_verts[i];
		Point_2 p1 = s.vertex(0);
		Point_2 p2 = s.vertex(1);
		pts.push_back(Point(to_double(p1.x()), to_double(p1.y())));
		handles.push_back(cdt.insert(pts.back()));
	}

	for (int i = 0; i < handles.size(); i++) {
		Vertex_handle p1 = handles[i];
		Vertex_handle p2 = handles[(i+1)%pts.size()];
		cdt.insert_constraint(p1, p2);
	}

	//CGAL::make_conforming_Delaunay_2(cdt);
	CGAL::refine_Delaunay_mesh_2(cdt, Criteria());
	vec2f p1, p2, p3;
	for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
		fit != cdt.finite_faces_end(); ++fit){
		if ((fit->is_in_domain())) {
			p1 = vec2f(cdt.triangle(fit)[0].x(), cdt.triangle(fit)[0].y());
			p2 = vec2f(cdt.triangle(fit)[1].x(), cdt.triangle(fit)[1].y());
			p3 = vec2f(cdt.triangle(fit)[2].x(), cdt.triangle(fit)[2].y());
			break;
		}
	}

	return (p1 + p2 + p3) / 3.0f; // Centroid of the triangle
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