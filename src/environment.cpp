#include <sys/types.h>
#include <sys/stat.h>
#include <direct.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <time.h>
#include <iomanip>

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

std::vector<Segment_2> environment::get_cgal_verts() {
	return cgal_verts;
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

float environment::get_closest_obstacle_distance(vec2f pt) {
	float closest = math::max_float;
	for (wall* w : walls) {
		vec2f* wall_p1 = w->get_vertices()[0];
		vec2f* wall_p2 = w->get_vertices()[1];
		float d = geom::line_point_distance(wall_p1, wall_p2, &pt);
		if (d < closest) closest = d;
	}

	for (obstacle* o : obstacles) {
		for (wall* w : o->get_walls()) {
			vec2f* wall_p1 = w->get_vertices()[0];
			vec2f* wall_p2 = w->get_vertices()[1];
			float d = geom::line_point_distance(wall_p1, wall_p2, &pt);
			if (d < closest) closest = d;
		}
	}

	return closest;
}

void environment::build_cgal_data() {
	// Environment boundary
	for (int i = 0; i < verts.size(); i++) {
		Point_2 p1(verts[i]->x, verts[i]->y);
		Point_2 p2(verts[(i + 1) % verts.size()]->x,
				   verts[(i + 1) % verts.size()]->y);
		cgal_verts.push_back(Segment_2(p1, p2));
		cgal_outP.push_back(p1);
	}

	// Holes (obstacles) in environment
	for (obstacle* o : obstacles) {
		cgal_holesP.push_back(Polygon_2());
		for (int i = 0; i < o->get_vertices().size(); i++) {
			Point_2 p1(o->get_vertices()[i]->x,
					   o->get_vertices()[i]->y);
			Point_2 p2(o->get_vertices()[(i + 1) % o->get_vertices().size()]->x,
					   o->get_vertices()[(i + 1) % o->get_vertices().size()]->y);
			cgal_verts.push_back(Segment_2(p1, p2));
			Point_2 p(o->get_vertices()[o->get_vertices().size() - 1 - i]->x,
					  o->get_vertices()[o->get_vertices().size() - 1 - i]->y);
			cgal_holesP.back().push_back(p);
		}
	}

	cgal_pwh = Polygon_with_holes_2(cgal_outP, cgal_holesP.begin(), cgal_holesP.end());
}

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
CDT environment::triangulate() {
	typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
	typedef CDT::Vertex_handle Vertex_handle;
	typedef CDT::Point Point;
	CDT cdt;

	// Add all the environment wall segments (and holes) as constraints
	for (Segment_2 s : cgal_verts) {
		Point source = Point(to_double(s.source().x()), to_double(s.source().y()));
		Point target = Point(to_double(s.target().x()), to_double(s.target().y()));
		Vertex_handle source_handle = cdt.insert(source);
		Vertex_handle target_handle = cdt.insert(target);
		cdt.insert_constraint(source_handle, target_handle);
	}

	// Specify seeds inside the obstacles, so that we don't triangulate them
	std::list<Point> list_of_seeds;
	for (obstacle* o : obstacles) {
		vec2f p = o->sample_point_inside();
		list_of_seeds.push_back(Point(p.x, p.y));
	}

	// Make the triangulated mesh
	CGAL::refine_Delaunay_mesh_2(cdt, list_of_seeds.begin(), list_of_seeds.end(),
		Criteria(), false);

	return cdt;
}

void environment::compute_features(char* name) {
	compute_area();
	std::vector<vec2f> sampled_points;
	CDT cdt = triangulate();
	int points_sampled_so_far = 0;

	// Samples points evenly across the environment by sampling points in the
	// environment mesh triangles.
	for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
		fit != cdt.finite_faces_end(); ++fit) {
		if ((fit->is_in_domain())) {
			vec2f p1 = vec2f(cdt.triangle(fit)[0].x(), cdt.triangle(fit)[0].y());
			vec2f p2 = vec2f(cdt.triangle(fit)[1].x(), cdt.triangle(fit)[1].y());
			vec2f p3 = vec2f(cdt.triangle(fit)[2].x(), cdt.triangle(fit)[2].y());

			float triangle_area = geom::polygon_area(std::vector<vec2f*> { &p1, & p2, & p3 });
			int points_to_sample = (int)((triangle_area / area) * NUM_POINTS_TO_SAMPLE);
			points_sampled_so_far += points_to_sample;
			if (points_sampled_so_far > NUM_POINTS_TO_SAMPLE) {
				points_to_sample -= points_sampled_so_far - NUM_POINTS_TO_SAMPLE;
				points_sampled_so_far = NUM_POINTS_TO_SAMPLE;
			}

			for (int i = 0; i < points_to_sample; i++) {
				sampled_points.push_back(geom::sample_triangle(&p1, &p2, &p3));
				assert(point_is_legal(sampled_points.back()));
			}
		}
	}
	for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
		fit != cdt.finite_faces_end(); ++fit) {
		int points_remaining_to_sample = NUM_POINTS_TO_SAMPLE - points_sampled_so_far;
		if ((fit->is_in_domain())) {
			vec2f p1 = vec2f(cdt.triangle(fit)[0].x(), cdt.triangle(fit)[0].y());
			vec2f p2 = vec2f(cdt.triangle(fit)[1].x(), cdt.triangle(fit)[1].y());
			vec2f p3 = vec2f(cdt.triangle(fit)[2].x(), cdt.triangle(fit)[2].y());

			float triangle_area = geom::polygon_area(std::vector<vec2f*> { &p1, & p2, & p3 });
			int points_to_sample = ceil((triangle_area / area) * points_remaining_to_sample);
			points_sampled_so_far += points_to_sample;
			if (points_sampled_so_far > NUM_POINTS_TO_SAMPLE) {
				points_to_sample -= points_sampled_so_far - NUM_POINTS_TO_SAMPLE;
				points_sampled_so_far = NUM_POINTS_TO_SAMPLE;
			}

			for (int i = 0; i < points_to_sample; i++) {
				sampled_points.push_back(geom::sample_triangle(&p1, &p2, &p3));
				assert(point_is_legal(sampled_points.back()));
			}
		}
	}

	// For each sampled point, compute the visibility polygon and all vis poly
	// features.
	std::vector<visibility_polygon*> vis_polys;
	for (vec2f p : sampled_points) {
		Polygon_2 cur_vis_poly = compute_vis_poly(p);
		std::vector<vec2f*> pts;
		for (auto vi = cur_vis_poly.vertices_begin(); vi != cur_vis_poly.vertices_end(); ++vi) {
			float px = CGAL::to_double((*vi).x());
			float py = CGAL::to_double((*vi).y());
			pts.push_back(new vec2f(px, py));
		}
		vis_polys.push_back(new visibility_polygon(pts, new vec2f(p.x, p.y), 0.0f));

		//for (vec2f* v : pts) {
			//delete v;
		//}
	}

	// Export the sample data (point location and features) to a text file.
	export_visibility_polygon_data(vis_polys, name);
	for (visibility_polygon* p : vis_polys) {
		delete p;
	}
}

void environment::compute_area() {
	float boundary_area = geom::polygon_area(verts);
	float obstacles_area = 0.0f;

	for (obstacle* o : obstacles) {
		obstacles_area += geom::polygon_area(o->get_vertices());
	}

	area = boundary_area - obstacles_area;
}

Polygon_2 environment::compute_vis_poly(vec2f pos) {
	// insert geometry into the arrangement
	Arrangement_2 arr_env;
	std::vector<Segment_2> verts = this->get_cgal_verts();
	CGAL::insert_non_intersecting_curves(arr_env, verts.begin(), verts.end());

	// find the face of the query point
	// (usually you may know that by other means)
	Point_2 query_point = Point_2(pos.x, pos.y);
	Arrangement_2::Face_const_handle* face;
	CGAL::Arr_naive_point_location<Arrangement_2> pl(arr_env);
	CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(query_point);
	// The query point locates in the interior of a face
	face = boost::get<Arrangement_2::Face_const_handle>(&obj);

	//visibility query
	Arrangement_2 output_arr;
	TEV tev(arr_env);
	Face_handle fh = tev.compute_visibility(query_point, *face, output_arr);
	Polygon_2 p;
	for (auto it = output_arr.vertices_begin(); it != output_arr.vertices_end(); it++) {
		p.push_back(it.ptr()->point());
	}
	return p;
}

void environment::export_visibility_polygon_data(std::vector<visibility_polygon*> vis_polys, char* name) {
	std::cout.precision(10);

	struct stat info;
	if (stat(config::DATA_DIR, &info) != 0)
		printf("cannot access %s, so the directory was created.\n", config::DATA_DIR);
	if (!(info.st_mode & S_IFDIR))  // S_ISDIR() doesn't exist on my windows 
		_mkdir(config::DATA_DIR);

	time_t t = std::time(0);
	struct tm* now = localtime(&t);

	char formatted_time[200];
	char temp[50];
	char filename[300];
	strcpy(filename, config::DATA_DIR);
	sprintf(temp, "/vis poly stats %s ", name);
	strcat(filename, temp);
	strftime(formatted_time, 200, "%Y-%m-%d-%H-%M-%S.csv", now);
	strcat(filename, formatted_time);

	std::ofstream my_file;
	my_file.open(filename);
	if (my_file.is_open()) {
		my_file << "point_location_x,point_location_y,num_verts,area,perimeter,min_dist,max_dist,avg_dist,jaggedness,vert_density\n";
		for (int i = 0; i < vis_polys.size(); i++) {
			visibility_polygon* poly = vis_polys[i];

			my_file << poly->center->x << ",";
			my_file << poly->center->y << ",";
			my_file << poly->num_vertices << ",";
			my_file << poly->area << ",";
			my_file << poly->perimeter << ",";
			my_file << poly->min_dist << ",";
			my_file << poly->max_dist << ",";
			my_file << poly->avg_dist << ",";
			my_file << poly->jaggedness << ",";
			my_file << poly->vertex_density;
			my_file << "\n";
		}
	}
	my_file.close();
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