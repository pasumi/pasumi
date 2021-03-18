#include "visibility_polygon.h"
#include "polygon_match.h"
#include "obstacle.h"
#include "wall.h"
#include "math.hpp"

//struct slice s;

visibility_polygon::visibility_polygon() {

}

visibility_polygon::visibility_polygon(std::vector<vec2f*> vertices, vec2f* center_pos, float _heading) {
	boundary_pts = vertices;
	//verts = vertices;
	center = center_pos;
	for (vec2f* v : vertices) {
		verts.push_back(new vec2f(v->x - center_pos->x, v->y - center_pos->y));
	}
	this->heading = _heading;
	distance_north = math::max_float;
	distance_east = math::max_float;
	distance_south = math::max_float;
	distance_west = math::max_float;
	compute_slices();
	flag = 0;
	//compute_features();
	//simplify();
	//normalize_polygon();
	//compute_turning_function();
}

visibility_polygon::visibility_polygon(environment* env_ptr, vec2f* center_pos) {
	env = env_ptr;
	center = center_pos;
	distance_north = math::max_float;
	distance_east = math::max_float;
	distance_south = math::max_float;
	distance_west = math::max_float;
	flag = 1;
	//compute_features();
	//simplify();
	//normalize_polygon();
	//compute_turning_function();
}

visibility_polygon::~visibility_polygon() {
	for (auto v : verts) {
		delete v;
	}

	for (auto s : slices) {
		delete s;
	}
}

void visibility_polygon::simplify() {
	std::vector<vec2f*> new_verts;
	std::vector<vec2f*> to_remove;

	for (int i = 0; i < boundary_pts.size(); i++) {
		vec2f* v1 = boundary_pts[i];
		vec2f* v2 = boundary_pts[(i + 1) % boundary_pts.size()];
		vec2f* v3 = boundary_pts[(i + 2) % boundary_pts.size()];

		// Remove the middle point if all 3 are collinear.
		if (geom::orient(v1, v2, v3) == 0) {
			to_remove.push_back(v2);
		}
	}
	for (vec2f* original_vert : boundary_pts) {
		bool okay = true;
		for (vec2f* bad_vert : to_remove) {
			if (*original_vert == *bad_vert) okay = false;
		}
		if (okay) new_verts.push_back(original_vert);
	}

	boundary_pts = new_verts;
}

void visibility_polygon::add_heading(float dir) {
	this->heading = fmod(dir, math::two_pi);
}

void visibility_polygon::compute_slices() {
	simple_slice();
	//smart_slice();
}

void visibility_polygon::simple_slice() {
	for (int i = 0; i < verts.size(); i++) {
		vec2f p1 = vec2f(verts[i]->x, verts[i]->y);
		vec2f p2 = vec2f(verts[(i + 1) % verts.size()]->x,
						 verts[(i + 1) % verts.size()]->y);

		vec2f canonical_p1 = vec2f(boundary_pts[i]->x, boundary_pts[i]->y);
		vec2f canonical_p2 = vec2f(boundary_pts[(i + 1) % boundary_pts.size()]->x,
								   boundary_pts[(i + 1) % boundary_pts.size()]->y);

		if (geom::orient(&p1, &p2, &(vec2f(0, 0))) == 0 ||
			signed_angle(normalize(p1), normalize(p2)) <= SIMPLE_SLICE_THETA_THRESHOLD) 
			continue; // colinear
		std::vector<vec2f*> arr;/* = new std::vector<vec2f*>;*/
		std::vector<vec2f*> canonical_arr;/* = new std::vector<vec2f*>;*/
		arr.push_back(new vec2f(p1.x, p1.y));
		arr.push_back(new vec2f(p2.x, p2.y));
		canonical_arr.push_back(new vec2f(canonical_p1.x, canonical_p1.y));
		canonical_arr.push_back(new vec2f(canonical_p2.x, canonical_p2.y));
		auto temp = new slice(&arr, &canonical_arr, heading);
		slices.push_back(temp);
		for (auto p : arr) {
			delete p;
		}
		for (auto p : canonical_arr) {
			delete p;
		}
	}
	std::sort(slices.begin(), slices.end(), compare_by_offset());
}

void visibility_polygon::smart_slice() {
	int start_index = 0;
	for (int end_index = 0; end_index < verts.size(); end_index++) {
		vec2f* p1 = verts[start_index];
		vec2f* p2 = verts[end_index % verts.size()];
		if (geom::orient(p1, p2, &(vec2f(0, 0))) == 0) continue; // colinear

		float angle_between_pts = angle(normalize(*p1), normalize(*p2));
		if (math::abs(angle_between_pts) < SLICE_THETA_THRESHOLD) continue; // Slice is too narrow

		std::vector<vec2f*>* arr = new std::vector<vec2f*>;
		for (int i = start_index; i <= end_index; i++) {
			arr->push_back(new vec2f(*(verts[i])));
		}
		//slices.push_back(new slice(arr, heading));
		start_index = end_index;
	}
	// Check the last slice manually
	vec2f* p1 = verts[start_index];
	vec2f* p2 = verts[0];
	float angle_between_pts = angle(normalize(*p1), normalize(*p2));
	if (geom::orient(p1, p2, &(vec2f(0, 0))) != 0 &&
		math::abs(angle_between_pts) >= SLICE_THETA_THRESHOLD) {
		std::vector<vec2f*>* arr = new std::vector<vec2f*>;
		arr->push_back(new vec2f(*p1));
		arr->push_back(new vec2f(*p2));
		//slices.push_back(new slice(arr, heading));
	}
	std::sort(slices.begin(), slices.end(), compare_by_offset());
}

void visibility_polygon::compute_features() {
	num_vertices = verts.size();
	area = geom::polygon_area(verts);
	perimeter = geom::polygon_perimeter(verts);
	min_dist = get_min_dist();
	max_dist = get_max_dist();
	avg_dist = area / perimeter;
	jaggedness = (perimeter * perimeter) / area;
	vertex_density = num_vertices / area;
}

float visibility_polygon::get_min_dist() {
	float best_dist = math::max_float;

	for (int i = 0; i < boundary_pts.size(); i++) {
		vec2f* p1 = boundary_pts[i];
		vec2f* p2 = boundary_pts[(i + 1) % boundary_pts.size()];
		float d = geom::line_point_distance(p1, p2, center);
		if (d < best_dist) best_dist = d;
	}

	return best_dist;
}

float visibility_polygon::get_max_dist() {
	float best_dist = -1.0f;

	for (int i = 0; i < boundary_pts.size(); i++) {
		vec2f* p1 = boundary_pts[i];
		float d = length(*center - *p1);
		if (d >= best_dist) best_dist = d;
	}

	return best_dist;
}

void visibility_polygon::normalize_polygon() {
	float perimeter = 0.0f;
	for (int i = 0; i < boundary_pts.size(); i++) {
		vec2f* p1 = boundary_pts[i];
		vec2f* p2 = boundary_pts[(i+1) % boundary_pts.size()];
		perimeter += length(*p1 - *p2);
	}
	normalized_verts.push_back(vec2f(0.0f, 0.0f));
	for (int i = 0; i < boundary_pts.size() - 1; i++) {
		vec2f* p1 = boundary_pts[i];
		vec2f* p2 = boundary_pts[(i + 1) % boundary_pts.size()];
		vec2f seg = *p2 - *p1;
		float seg_length = length(seg);
		vec2f new_seg = normalize(seg) * (seg_length / perimeter);
		normalized_verts.push_back(normalized_verts[i] + new_seg);
	}
}

polygon_similarity visibility_polygon::compare(visibility_polygon other_poly) {
	poly poly_f, poly_g;
	TURN_REP_REC trf, trg;
	TURN_REP f, g;
	POLY_REC pf, pg;
	EVENT_REC e;
	double ht0, slope, alpha, theta_star, metric2, metric, ht0_err, slope_err;
	int update_p = 1;
	int precise_p = 0;
	int i = 0;
	while (i < verts.size()) {
		poly_f.pt[i].x = verts[i]->x;
		poly_f.pt[i].y = verts[i]->y;
		i++;
	}
	poly_f.n = i;

	i = 0;
	while (i < other_poly.verts.size()) {
		poly_g.pt[i].x = other_poly.verts[i]->x;
		poly_g.pt[i].y = other_poly.verts[i]->y;
		i++;
	}
	poly_g.n = i;

	poly_to_turn_rep(&poly_g, &trg);
	g = &trg;
	poly_to_turn_rep(&poly_f, &trf);
	f = &trf;

	init_vals(f, g, &ht0, &slope, &alpha);
	init_events(f, g);
	metric2 = h_t0min(f, g, ht0, slope, alpha, update_p ? reinit_interval(f, g) : 0, &theta_star, &e, &ht0_err, &slope_err);
	metric = metric2 > 0 ? sqrt(metric2) : 0;

	return polygon_similarity(metric, turn(theta_star, 0));
}

visibility_polygon visibility_polygon::rotate(vec2f rotation_origin, float theta) {
	std::vector<vec2f*> rotated_verts;
	for (vec2f* v : boundary_pts) {
		rotated_verts.push_back(new vec2f(rotate_around(rotation_origin, *v, theta)));
	}
	return visibility_polygon(rotated_verts, center, 0.0f);
}

float visibility_polygon::compute_distance_loss(visibility_polygon* virt_poly, float phys_heading, float virt_heading) {
	this->compute_nearest_features(rad_2_vec(phys_heading));
	virt_poly->compute_nearest_features(rad_2_vec(virt_heading));

	float sum = 0.0f;
	sum += math::abs(distance_north - virt_poly->distance_north);
	sum += math::abs(distance_east - virt_poly->distance_east);
	sum += math::abs(distance_south - virt_poly->distance_south);
	sum += math::abs(distance_west - virt_poly->distance_west);
	return sum;
}

/*
THIS IS THE VERSION OF THE FUNCTION THAT WORKED FOR WHEN I WAS COMPUTING THE VISIBILITY POLYGON. IT IS STILL USEFUL IF I WANT TO SWITCH BACK TO THAT VERSION OF THE CODE.
void visibility_polygon::compute_nearest_features(vec2f heading) {
	std::vector<vec2f> directions{ heading, vec2f(heading.y, -heading.x), vec2f(-heading.x, -heading.y), vec2f(-heading.y, heading.x) };

	for (int i = 0; i < directions.size(); i++) {
		for (int j = 0; j < verts.size(); j++) {
			vec2f p1 = verts[j];
			vec2f p2 = verts[(j + 1) % verts.size()];
			float t = geom::ray_line_intersect(center, directions[i], p1, p2);
			if (t >= 0) {
				switch (i) {
				case 0: {
					//if (t < distance_north) 
					distance_north = t;
					break;
				}
				case 1: {
					//if (t < distance_east) 
					distance_east = t;
					break;
				}
				case 2: {
					//if (t < distance_south) 
					distance_south = t;
					break;
				}
				case 3: {
					//if (t < distance_west) 
					distance_west = t;
					break;
				}
				}
			}
			if (t >= 0) {
				//break;
			}

		}
	}
}
*/

void visibility_polygon::compute_nearest_features(vec2f heading) {
	std::vector<vec2f> directions{ heading, vec2f(heading.y, -heading.x), vec2f(-heading.x, -heading.y), vec2f(-heading.y, heading.x) };

	for (int i = 0; i < directions.size(); i++) {
		float closest = math::max_float;
		for (int j = 0; j < boundary_pts.size(); j++) {
			vec2f* p1 = boundary_pts[j];
			vec2f* p2 = boundary_pts[(j+1)%boundary_pts.size()];
			float t = geom::ray_line_intersect(center, new vec2f(directions[i]), p1, p2);
			if (t >= 0 && t < closest) {
				closest = t;
			}
		}

		// Assign the distance value to the correct direction.
		switch (i) {
		case 0: {
			distance_north = closest;
			break;
		}
		case 1: {
			distance_east = closest;
			break;
		}
		case 2: {
			distance_south = closest;
			break;
		}
		case 3: {
			distance_west = closest;
			break;
		}
		}
	}
}

float visibility_polygon::operator-(const visibility_polygon& o) {
	float sum = 0.0f;
	sum += math::abs(this->num_vertices - o.num_vertices);
	sum += math::abs(this->area - o.area);
	sum += math::abs(this->perimeter - o.perimeter);
	sum += math::abs(this->min_dist - o.min_dist);
	sum += math::abs(this->max_dist - o.max_dist);
	sum += math::abs(this->avg_dist - o.avg_dist);
	sum += math::abs(this->jaggedness - o.jaggedness);
	sum += math::abs(this->vertex_density - o.vertex_density);
	return sum;
}

// Check if the point v is a part of the visibility polygon
bool visibility_polygon::is_vert_in(vec2f v) {
	for (vec2f* vert : verts) {
		if (*vert == v) {
			return true;
		}
	}
	return false;
}