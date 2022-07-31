#pragma once

#include <cassert>

class environment;

#include "vec2f.h"
#include "geometry.h"
#include "polygon.h"

//#include "math.hpp"
//#include <CGAL/Point_2.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/number_utils.h>
//#include <CGAL/Cartesian.h>
//typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
//typedef Kernel::Point_2                                         Point_2;
//typedef CGAL::Point_2<CGAL::Cartesian<double>> Point;

struct vis_poly_edge;
struct vis_poly_vertex;
struct edge_wapper;

struct vis_poly_vertex {
	vis_poly_vertex() {

	}

	vis_poly_vertex(float new_r, float new_theta) {
		r = new_r;
		theta = new_theta;
		cartesian = vec2f(r * math::cos(theta), r * math::sin(theta));
		e = nullptr;
	}

	vis_poly_vertex(vec2f pos, vec2f cartesian_origin, bool first) {
		cartesian = pos;
		origin = cartesian_origin;
		vec2f p = polar(origin, pos);
		r = p.x;
		theta = p.y;
		e = nullptr;
		is_first = first;
	}

	inline bool operator== (const vis_poly_vertex& p2) {
		bool b1 = r == p2.r;
		bool b2 = theta == p2.theta;
		bool b3 = cartesian == p2.cartesian;
		return ((r == p2.r) && (theta == p2.theta) && (cartesian == p2.cartesian));
	}

	float r;
	float theta;
	vec2f cartesian;
	vec2f origin;
	vis_poly_edge* e;
	bool is_first;
};

struct vis_poly_edge {
	vis_poly_edge() {

	}

	vis_poly_edge(vis_poly_vertex* start, vis_poly_vertex* end) {
		p1 = start;
		p2 = end;
		p1->e = this;
	}

	vis_poly_vertex* p1;
	vis_poly_vertex* p2;
};

struct polygon_similarity {
	polygon_similarity();

	polygon_similarity(float sim_metric, float amount_to_rotate) {
		metric = sim_metric;
		theta = amount_to_rotate;
	}

	float metric;
	float theta;
};

struct slice {
	slice() {}

	slice(std::vector<vec2f*>* _pts, float heading) {
		pts = new std::vector<vec2f*>;
		for (vec2f* v : *_pts) {
			pts->push_back(new vec2f(*v));
		}
		pts->push_back(new vec2f(0.0f, 0.0f));
		
		p1 = (*pts)[0];
		p2 = (*pts)[pts->size() - 2]; // Skip the kernel point (0, 0) aka the slice tip

		p1_theta = signed_angle(rad_2_vec(heading), normalize(*p1));
		p2_theta = signed_angle(rad_2_vec(heading), normalize(*p2));

		vec2f p1_to_p2 = *p2 - *p1;
		bisector = normalize(*p1 + (p1_to_p2 * 0.5f));
		theta_offset = angle(rad_2_vec(heading), bisector);
		width = math::abs(angle(normalize(*p1), normalize(*p2)));
		area = geom::polygon_area(*pts);
		avg_height = 0.0f;
		for (vec2f* v : *pts){
			avg_height += length(*v);
		}
		avg_height = avg_height / (pts->size() - 1); // minus 1 to skip the center point (0, 0)
		//height = geom::ray_line_intersect(pts->back(), &bisector, p1, p2);
		//assert(height != -1.0f, "Cannot calculate height of slice!");
		assert(avg_height > 0.0f, "Average height is zero!");
	}

	std::vector<vec2f*>* pts;
	vec2f* p1;
	vec2f* p2;
	float p1_theta;
	float p2_theta;
	vec2f bisector;
	float theta_offset;
	float width;
	float area;
	float height;
	float avg_height;
};

struct compare_by_offset {
	inline bool operator() (const slice* a, const slice* b) {
		return a->theta_offset < b->theta_offset;
	}
};

class visibility_polygon : public polygon {
	public:
		visibility_polygon();
		visibility_polygon(std::vector<vec2f*> vertices, vec2f* center, float _heading);
		visibility_polygon(environment* env_ptr, vec2f* center);
		//~visibility_polygon();
		void simplify();
		void compute_slices();

		std::vector<vec2f*> verts;
		std::vector<slice*> slices;
		environment* env;
		vec2f* center;

		// Polygon features
		int num_vertices;
		float area;
		float perimeter;
		float heading;

	private:
		float SLICE_THETA_THRESHOLD = 0.0174533f; // 30 degrees
};