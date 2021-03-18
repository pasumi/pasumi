#include <algorithm>

#include "edge_pq.h"

edge_pq::edge_pq() {

}

edge_pq::edge_pq(vec2f o) {
	origin = o;
}

void edge_pq::set_target(vec2f new_target) {
	target = new_target;
}

void edge_pq::insert(vis_poly_edge* new_edge) {
	float t = geom::ray_line_intersect(&origin, &dir, &(new_edge->p1->cartesian), &(new_edge->p2->cartesian));
	std::pair<vis_poly_edge*, float> e = std::make_pair(new_edge, t);

	intersecting_edges.push_back(e);
	std::sort(intersecting_edges.begin(), intersecting_edges.end(), edge_intersection_comparator());
}

std::pair<vis_poly_edge*, float> edge_pq::pop() {
	std::pair<vis_poly_edge*, float> retval = intersecting_edges[0];
	intersecting_edges.erase(intersecting_edges.begin());
	return retval;
}

void edge_pq::sort() {
	edges.clear();
	for (auto e : intersecting_edges) {
		edges.push_back(e.first);
	}
	intersecting_edges.clear();
	dir = normalize(target - origin);

	for (vis_poly_edge* e : edges) {
		vec2f p1 = e->p1->cartesian;
		vec2f p2 = e->p2->cartesian;
		vec2f p1_ext = normalize(p1 - p2) * math::epsilon;
		vec2f p2_ext = normalize(p2 - p1) * math::epsilon;
		p1 += p1_ext;
		p2 += p2_ext;

		float t = geom::ray_line_intersect(&origin, &dir, &p1, &p2);
		if (t >= 0) {
			intersecting_edges.push_back(std::make_pair(e, t));
		}
	}

	std::sort(intersecting_edges.begin(), intersecting_edges.end(), edge_intersection_comparator());
}