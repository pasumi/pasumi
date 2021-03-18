#pragma once

#include "visibility_polygon.h"

struct edge_intersection_comparator {
	inline bool operator() (const std::pair<vis_poly_edge*, float>& e1, const std::pair<vis_poly_edge*, float>& e2) {
		return e1.second < e2.second;
	}
};

class edge_pq {
	public:
		edge_pq();
		edge_pq(vec2f o);
		void set_target(vec2f new_target);
		void insert(vis_poly_edge* new_edge);
		void sort();
		std::pair<vis_poly_edge*, float> pop();

		vec2f origin;
		vec2f target;
		vec2f dir;
		std::vector<std::pair<vis_poly_edge*, float>> intersecting_edges;
		std::vector<vis_poly_edge*> edges;

	private:

};