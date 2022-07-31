#include "visibility_polygon.h"
#include "obstacle.h"
#include "wall.h"
#include "math.hpp"

struct slice s;

visibility_polygon::visibility_polygon() {

}

visibility_polygon::visibility_polygon(std::vector<vec2f*> vertices, vec2f* center_pos, float _heading) {
	boundary_pts = vertices;
	center = center_pos;
	for (vec2f* v : vertices) {
		verts.push_back(new vec2f(v->x - center_pos->x, v->y - center_pos->y));
	}
	this->heading = _heading;
	compute_slices();
	//simplify();
}

visibility_polygon::visibility_polygon(environment* env_ptr, vec2f* center_pos) {
	env = env_ptr;
	center = center_pos;
	//simplify();
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

void visibility_polygon::compute_slices() {
	for (int i = 0; i < verts.size(); i++) {
		vec2f* p1 = verts[i];
		vec2f* p2 = verts[(i + 1) % verts.size()];
		if (geom::orient(p1, p2, &(vec2f(0, 0))) == 0 ||
			signed_angle(normalize(*p1), normalize(*p2)) <= SLICE_THETA_THRESHOLD) continue; // colinear
		std::vector<vec2f*>* arr = new std::vector<vec2f*>;
		arr->push_back(new vec2f(*p1));
		arr->push_back(new vec2f(*p2));
		slices.push_back(new slice(arr, heading));
	}
	std::sort(slices.begin(), slices.end(), compare_by_offset());
}

