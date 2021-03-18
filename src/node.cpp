#include <cassert>
#include <algorithm>
#include <queue> 
#include <utility>

#include "node.h"
#include "edge.h"
#include "geometry.h"
#include "edge_pq.h"

#include "visibility.hpp"
#include "vector2.hpp"

node::node(vec2f location) {
	this->location = location;
	distance_north = math::max_float;
	distance_east = math::max_float;
	distance_south = math::max_float;
	distance_west = math::max_float;
}

bool node::add_neigbor(node* n) {
	return this->neigbors.insert(n).second;
}

bool node::add_edge(edge* e) {
	return this->edges.insert(e).second;
}

void node::compute_visibility_polygon(environment* env) {
	// https://github.com/trylock/visibility
	using vector_type = geometry::vec2;
	using segment_type = geometry::line_segment<vector_type>;
	using segment_comparer_type = geometry::line_segment_dist_comparer<vector_type>;
	using angle_comparer_type = geometry::angle_comparer<vector_type>;
	using namespace geometry;

	std::vector<segment_type> segments;
	for (int i = 0; i < env->get_vertices().size(); i++) {
		vec2f* p1 = env->get_vertices()[i];
		vec2f* p2 = env->get_vertices()[(i + 1) % env->get_vertices().size()];
		segments.push_back({ {p1->x, p1->y}, {p2->x, p2->y} });
	}
	for (obstacle* o : env->get_obstacles()) {
		for (int i = 0; i < o->get_vertices().size(); i++) {
			vec2f* p1 = o->get_vertices()[i];
			vec2f* p2 = o->get_vertices()[(i + 1) % o->get_vertices().size()];
			segments.push_back({ {p1->x, p1->y}, {p2->x, p2->y} });
		}
	}

	auto poly = visibility_polygon2(vector_type{ location.x, location.y }, segments.begin(), segments.end());
	std::vector<vec2f*> temp;
	for (vec2 p : poly) {
		temp.insert(temp.begin(), new vec2f(p.x, p.y)); // Want it in CCW order
	}
	vis_polygon = visibility_polygon(temp, &location, 0.0f);
}

void node::update_features(std::set<node*> all_nodes) {
	neighborhood_size = neigbors.size();

	std::set<edge*> cluster_edges;
	for (node* n : neigbors) {
		for (edge* e : n->edges) {
			if (neigbors.find(e->n1) != neigbors.end() &&
				neigbors.find(e->n2) != neigbors.end()) {
				cluster_edges.insert(e);
			}
		}
	}
	clustering_coeff = (cluster_edges.size() * 2) / (neighborhood_size * (neighborhood_size - 1));

	int path_length_sum = 0.0f;
	for (node* destination : all_nodes) {
		if (destination != this) {
			path_length_sum += get_shortest_path_length(this, destination, all_nodes);
		}
	}
	mean_shortest_path_length = (float)path_length_sum / all_nodes.size();

	//compute_nearest_features(vec2f(0.0f, 1.0f));
}

int node::get_shortest_path_length(node* source, node* destination, std::set<node*> all_nodes) {
	int length = 0;
	std::priority_queue<node*> pq;
	source->visited = true;
	pq.push(source);

	node* cur;
	while (!pq.empty()) {
		cur = pq.top();
		pq.pop();
		if (cur == destination) {
			length++;
			break;
		}
		for (node* neighbor : cur->neigbors) {
			if (!neighbor->visited) {
				neighbor->visited = true;
				pq.push(neighbor);
			}
		}
	}

	for (node* n : all_nodes) {
		n->visited = false;
	}

	return length;
}

void node::compute_nearest_features(vec2f heading) {
	std::vector<vec2f> directions{ heading, vec2f(heading.y, -heading.x), vec2f(-heading.x, -heading.y), vec2f(-heading.y, heading.x) };

	for (int i = 0; i < directions.size(); i++) {
		for (int j = 0; j < vis_polygon.verts.size(); j++) {
			vec2f* p1 = vis_polygon.verts[j];
			vec2f* p2 = vis_polygon.verts[(j + 1) % vis_polygon.verts.size()];
			float t = geom::ray_line_intersect(&location, &(directions[i]), p1, p2);
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
	int t = 4;
}

polygon_similarity node::compute_similarity(node* other_node, float phys_heading, float virt_heading) {
	// We want to rotate the virtual visibility polygon such that its orientation in
	// the similarity-measuring algorithm is how it would appear to the user standing
	// at that visibility node. The notion of "no rotation" (theta == 0) is different
	// for a user and a polygon. theta == 0 for a user is the vector pointing to the 
	// right. theta == 0 for a polygon is the vector pointing straight up.
	float virtual_angle_to_align = signed_angle(rad_2_vec(virt_heading), vec2f(0.0f, 1.0f));
	//virt_heading = math::clamp_rotation(virt_heading);
	//virt_heading -= math::pi / 2.0f;
	visibility_polygon virtual_rotated_virt_vis_poly = other_node->vis_polygon.rotate(vec2f(0.0f, 0.0f), virtual_angle_to_align);

	float physical_angle_to_align = signed_angle(rad_2_vec(phys_heading), vec2f(0.0f, 1.0f));
	visibility_polygon physical_rotated_virt_vis_poly = vis_polygon.rotate(vec2f(0.0f, 0.0f), physical_angle_to_align);

	//return vis_polygon.compare(rotated_virt_vis_poly);
	return physical_rotated_virt_vis_poly.compare(virtual_rotated_virt_vis_poly);
}

float node::compute_distance_loss(node* other_node, float phys_heading, float virt_heading) {
	this->compute_nearest_features(rad_2_vec(phys_heading));
	other_node->compute_nearest_features(rad_2_vec(virt_heading));

	float sum = 0.0f;
	sum += math::abs(distance_north - other_node->distance_north);
	sum += math::abs(distance_east - other_node->distance_east);
	sum += math::abs(distance_south - other_node->distance_south);
	sum += math::abs(distance_west - other_node->distance_west);
	return sum;
}

/*****************************************************************************
 *****************************************************************************
 *****************************************************************************
 *****************************************************************************

THIS CODE IS MY ATTEMPT AT THE VISIBILITY POLYGON ALGORITHM. I FAILED. I DON'T HAVE TIME
TO FIX IT, SO I WILL JUST LEAVE THIS HERE FOR ANOTHER DAY WHEN I WANT TO FIX IT. I USED
A RANDOM IMPLEMENTATION FROM GITHUB INSTEAD.


vis_poly_edge* node::get_active_edge(vis_poly_vertex* cur_vert, std::vector<vis_poly_edge*> edges) {
	vec2f dir = normalize(cur_vert->cartesian - location);
	float min_t = math::max_float;
	vis_poly_edge* closest_edge = NULL;
	for (vis_poly_edge* e : edges) {
		float t = geom::ray_line_intersect(location, dir, e->p1->cartesian, e->p2->cartesian);
		if (t != -1.0f) {
			if (t < min_t) {
				min_t = t;
				closest_edge = e;
			}
		}
	}
	assert(closest_edge != NULL);
	return closest_edge;
}

vis_poly_edge* node::get_next_edge(int cur_index, std::vector<vis_poly_vertex*> verts) {
	vis_poly_vertex* cur_vert = verts[cur_index];
	cur_index++;
	vis_poly_vertex* next_vert = verts[cur_index];
	while (next_vert->cartesian == cur_vert->cartesian) {
		cur_index++;
		next_vert = verts[cur_index];
	}
	return next_vert->e;
}

/* Converts a line segment in cartesian coordinates into polar coordinates.
 * Also prunes backfacing edges (edges which cannot possibly contribute to
 * the visibility polygon).
void node::process_edge(wall* w, std::vector<vis_poly_edge*>& edges, std::vector<vis_poly_vertex*>& polar_pts) {
	vec2f p1 = w->get_vertices()[0];
	vec2f p2 = w->get_vertices()[1];
	vec2f polar_p1 = polar(location, p1);
	vec2f polar_p2 = polar(location, p2);

	vis_poly_vertex* vp1 = new vis_poly_vertex(polar_p1.x, polar_p1.y);
	vis_poly_vertex* vp2 = new vis_poly_vertex(polar_p2.x, polar_p2.y);

	float t = geom::ray_line_intersect(location, vec2f(1, 0), p1, p2);
	// Split the wall into two segments if it crosses theta = 0 horizontal line.
	if (t != -1.0f) {
		vec2f p_mid = polar(location, vec2f(location.x + t, location.y));
		vis_poly_vertex* vp_mid1 = new vis_poly_vertex(p_mid.x, math::two_pi);
		vis_poly_vertex* vp_mid2 = new vis_poly_vertex(p_mid.x, 0.0f);


		if (vp1->theta < vp_mid1->theta) {
			vis_poly_edge* e1 = new vis_poly_edge(vp1, vp_mid1);
			edges.push_back(e1);
			polar_pts.push_back(vp1);
		}
		if (vp_mid2->theta < vp2->theta) {
			vis_poly_edge* e2 = new vis_poly_edge(vp_mid2, vp2);
			edges.push_back(e2);
			polar_pts.push_back(vp_mid2);
		}
	}
	// Otherwise, just add the point normally.
	else {
		if (vp1->theta < vp2->theta) {
			edges.push_back(new vis_poly_edge(vp1, vp2));
			polar_pts.push_back(vp1);
		}
	}
}

void node::handle_event_point(std::vector<vis_poly_vertex*>& polar_pts,
	int i, std::vector<vis_poly_vertex*>& vis_poly,
	vis_poly_edge* active_edge, std::priority_queue<vis_poly_vertex*>& event_queue) {
	vis_poly_vertex* cur_vert = polar_pts[i];
	vis_poly_edge* cur_edge = cur_vert->e;
	vis_poly_edge* next_edge = polar_pts[i + 1]->e;
	vec2f ray_dir = normalize(cur_vert->cartesian - location);

	if (cur_edge == active_edge) {
		if (cur_vert == active_edge->p1) return;
		if (next_edge->p1 == cur_edge->p2) {
			// Case 1
			vis_poly.push_back(cur_vert);
			active_edge = next_edge;
			return;
		}
	}
}

void node::extract_boundary_points_and_edges(std::vector<vis_poly_vertex*> verts, std::vector<vis_poly_vertex*>& polar_pts, std::vector<vis_poly_edge*>& edges) {
	// Get all the edges that may contribute to the final visibility polygon.
	// Prune edges that cannot possibly contribute to the final polygon.
	std::vector<vis_poly_vertex*> temp_midpoints;
	for (int i = 0; i < verts.size(); i++) {
		vis_poly_vertex* p1 = verts[i];
		vis_poly_vertex* p2 = verts[(i + 1) % verts.size()];

		float t = geom::ray_line_intersect(location, vec2f(1, 0), p1->cartesian, p2->cartesian);
		// Split the wall into two segments if it crosses theta = 0 horizontal line.
		if (t != -1.0f) {
			vec2f mid = vec2f(location.x + t, location.y);
			vis_poly_vertex* midpoint_end = new vis_poly_vertex(mid, location, true);
			midpoint_end->theta = math::two_pi;
			vis_poly_vertex* midpoint_start = new vis_poly_vertex(mid, location, true);

			if (p1->theta < midpoint_end->theta) {
				vis_poly_vertex* start = new vis_poly_vertex(p1->cartesian, location, true);
				vis_poly_vertex* end = new vis_poly_vertex(midpoint_end->cartesian, location, false);
				end->theta = midpoint_end->theta;

				vis_poly_edge* new_edge = new vis_poly_edge(start, end);
				start->e = new_edge;
				end->e = new_edge;

				polar_pts.push_back(start);
				polar_pts.push_back(end);
				edges.push_back(new_edge);
			}
			if (midpoint_start->theta < p2->theta) {
				vis_poly_vertex* start = new vis_poly_vertex(midpoint_start->cartesian, location, true);
				vis_poly_vertex* end = new vis_poly_vertex(p2->cartesian, location, false);

				vis_poly_edge* new_edge = new vis_poly_edge(start, end);
				start->e = new_edge;
				end->e = new_edge;

				polar_pts.push_back(end);
				polar_pts.push_back(start);
				edges.push_back(new_edge);
			}
		}
		// Otherwise, just add the point normally.
		else {
			if (p1->theta < p2->theta) {
				vis_poly_vertex* start = new vis_poly_vertex(p1->cartesian, location, true);
				vis_poly_vertex* end = new vis_poly_vertex(p2->cartesian, location, false);
				vis_poly_edge* new_edge = new vis_poly_edge(start, end);
				start->e = new_edge;
				end->e = new_edge;

				polar_pts.push_back(start);
				polar_pts.push_back(end);
				edges.push_back(new_edge);
			}
		}
	}
}

void node::extract_obstacle_points_and_edges(std::vector<vis_poly_vertex*> verts, std::vector<vis_poly_vertex*>& polar_pts, std::vector<vis_poly_edge*>& edges) {
	// Get all the edges that may contribute to the final visibility polygon.
	// Prune edges that cannot possibly contribute to the final polygon.
	std::vector<vis_poly_vertex*> temp_midpoints;
	for (int i = 0; i < verts.size(); i++) {
		vis_poly_vertex* p1 = verts[i];
		vis_poly_vertex* p2 = verts[(i + 1) % verts.size()];

		float t = geom::ray_line_intersect(location, vec2f(1, 0), p1->cartesian, p2->cartesian);
		// Split the wall into two segments if it crosses theta = 0 horizontal line.
		if (t != -1.0f) {
			// TODO: I THINK THE BUG IS HERE WITH MATCHING THE POINTS TO START AND END POINTS OF HTE EDGES FOR OBSTACLES. MAYBE NEED A SEPARATE FUNCITON FOR OBJECTS
			vec2f mid = vec2f(location.x + t, location.y);
			vis_poly_vertex* midpoint_end = new vis_poly_vertex(mid, location, true);
			vis_poly_vertex* midpoint_start = new vis_poly_vertex(mid, location, true);
			midpoint_start->theta = math::two_pi;

			if (p1->theta < midpoint_end->theta) {
				vis_poly_vertex* start = new vis_poly_vertex(p1->cartesian, location, true);
				vis_poly_vertex* end = new vis_poly_vertex(midpoint_end->cartesian, location, false);
				end->theta = midpoint_end->theta;

				vis_poly_edge* new_edge = new vis_poly_edge(start, end);
				start->e = new_edge;
				end->e = new_edge;

				polar_pts.push_back(start);
				polar_pts.push_back(end);
				edges.push_back(new_edge);
			}
			if (midpoint_start->theta < p2->theta) {
				vis_poly_vertex* start = new vis_poly_vertex(midpoint_start->cartesian, location, true);
				vis_poly_vertex* end = new vis_poly_vertex(p2->cartesian, location, false);

				vis_poly_edge* new_edge = new vis_poly_edge(start, end);
				start->e = new_edge;
				end->e = new_edge;

				polar_pts.push_back(end);
				polar_pts.push_back(start);
				edges.push_back(new_edge);
			}
		}
		// Otherwise, just add the point normally.
		else {
			if (p1->theta < p2->theta) {
				vis_poly_vertex* start = new vis_poly_vertex(p1->cartesian, location, true);
				vis_poly_vertex* end = new vis_poly_vertex(p2->cartesian, location, false);
				vis_poly_edge* new_edge = new vis_poly_edge(start, end);
				start->e = new_edge;
				end->e = new_edge;

				polar_pts.push_back(start);
				polar_pts.push_back(end);
				edges.push_back(new_edge);
			}
		}
	}
}

bool node::check_if_visible(vis_poly_edge* edge_to_check, vis_poly_edge* active_edge) {
	vec2f edge_to_check_p1 = edge_to_check->p1->cartesian;
	vec2f edge_to_check_p2 = edge_to_check->p2->cartesian;
	vec2f look_target_p1 = active_edge->p1->cartesian;
	vec2f look_target_p2 = active_edge->p2->cartesian;
	float t1 = geom::ray_line_intersect(location, normalize(look_target_p1 - location), edge_to_check_p1, edge_to_check_p2);
	float t2 = geom::ray_line_intersect(location, normalize(look_target_p2 - location), edge_to_check_p1, edge_to_check_p2);
	return (t1 >= 0 || t2 >= 0);
}

std::vector<std::pair<vis_poly_edge*, float>> node::get_intersecting_edges(vis_poly_vertex* cur_vert, std::vector<vis_poly_edge*> edges) {
	vec2f dir = normalize(cur_vert->cartesian - location);
	float min_t = math::max_float;
	std::vector<std::pair<vis_poly_edge*, float>> intersecting_edges;

	for (vis_poly_edge* e : edges) {
		vec2f p1 = e->p1->cartesian;
		vec2f p2 = e->p2->cartesian;
		vec2f p1_ext = normalize(p1 - p2) * math::epsilon;
		vec2f p2_ext = normalize(p2 - p1) * math::epsilon;
		p1 += p1_ext;
		p2 += p2_ext;
		float t = geom::ray_line_intersect(location, dir, p1, p2);
		if (t >= 0) {
			intersecting_edges.push_back(std::make_pair(e, t));
		}
	}

	std::sort(intersecting_edges.begin(), intersecting_edges.end(), edge_intersection_comparator());
	return intersecting_edges;
}

std::vector<vis_poly_vertex*> polar_pts;
std::vector<vis_poly_edge*> edges;
std::vector<vis_poly_vertex*> temp_pts;
location = vec2f(-2.0f, 0.0f); //TODO: remove this line. it was for debugging

// Convert all the points into polygon verts (for polar coordinate stuff).
for (int i = 0; i < env->get_walls().size(); i++) {
	vec2f p = env->get_walls()[i]->get_vertices()[0];
	temp_pts.push_back(new vis_poly_vertex(p, location, true));
}
extract_boundary_points_and_edges(temp_pts, polar_pts, edges);
temp_pts.clear();
for (obstacle* o : env->get_obstacles()) {
	// Need polygon holes' boundaries in CW order.
	for (int i = o->get_walls().size() - 1; i >= 0; i--) {
		vec2f p = o->get_walls()[i].get_vertices()[0];
		temp_pts.push_back(new vis_poly_vertex(p, location, true));
	}
	extract_obstacle_points_and_edges(temp_pts, polar_pts, edges);
	temp_pts.clear();
}

// Sort by polar angle.
assert(polar_pts.size() > 0);
std::sort(polar_pts.begin(), polar_pts.end(), polar_coord_comparator());

// Compute the visibility polygon with an angular line sweep.
// http://www.cs.bu.edu/techreports/pdf/2004-015-camera-layout.pdf
vis_poly_vertex* cur_vert = polar_pts[0];
vis_poly_edge* active_edge = cur_vert->e;
std::vector<vis_poly_vertex*> vis_poly{ cur_vert };
edge_pq pq = edge_pq(location);
for (int i = 1; i < polar_pts.size(); i++) {
	if (polar_pts[i]->theta == 0) {
		pq.insert(polar_pts[i]->e);
	}
}
for (int i = 1; i < polar_pts.size() - 1; i++) {
	cur_vert = polar_pts[i];
	vis_poly_edge* next_edge = polar_pts[i + 1]->e;
	std::vector<std::pair<vis_poly_edge*, float>> intersecting_edges = get_intersecting_edges(cur_vert, edges);
	pq.set_target(cur_vert->cartesian);
	pq.sort();

	// Current vertex is end vertex of the active edge
	if (cur_vert == active_edge->p2) {
		// Case 1: Next edge is contiguous with active edge
		if (next_edge->p1->cartesian == active_edge->p2->cartesian) {
			vis_poly.push_back(cur_vert);
			active_edge = next_edge;
			continue;
		}
		// Case 2: Next edge is not contiguous with active edge
		if (next_edge->p1->cartesian != active_edge->p2->cartesian) {
			std::pair<vis_poly_edge*, float> e = pq.pop();
			while (!check_if_visible(e.first, active_edge)) {
				e = pq.pop();
			}

			// The first intersecting edge is always the edge belonging to the current vertex, so we get the second closest intersecting edge.
			// TODO: test to make sure that the above comment is actually true.
			//std::pair<vis_poly_edge*, float> closest_edge = intersecting_edges[1];

			// Compute the k-point and add it to the visibility polygon.
			vec2f dir = normalize(cur_vert->cartesian - location);
			vec2f intersection_point = location + (dir * e.second);
			vis_poly.push_back(new vis_poly_vertex(intersection_point, location, false));
			vis_poly.push_back(cur_vert);

			active_edge = e.first;
		}
	}
	// Current vertex is start vertex of some edge
	else if (cur_vert->is_first) {
		std::pair<vis_poly_edge*, float> cur_vertex_edge_intersection;
		std::pair<vis_poly_edge*, float> active_edge_intersection;
		for (auto e : intersecting_edges) {
			if (e.first == cur_vert->e) {
				cur_vertex_edge_intersection = e;
			}
			if (e.first == active_edge) {
				active_edge_intersection = e;
			}
		}

		// Case 3: Current vertex belongs to an intersecting edge that is closer to 
		// the observer than the active edge is.
		if (cur_vert->e != active_edge &&
			cur_vertex_edge_intersection.second < active_edge_intersection.second) {
			// Compute the k-point and add it to the visibility polygon
			vec2f dir = normalize(cur_vert->cartesian - location);
			vec2f intersection_point = location + (dir * active_edge_intersection.second);
			vis_poly.push_back(new vis_poly_vertex(intersection_point, location, false));

			// The current vertex is also part of the visibility polygon
			vis_poly.push_back(cur_vert);

			pq.insert(active_edge);

			active_edge = cur_vert->e;
		}
		// Case 4: Current vertex belongs to an intersecting edge that is further 
		// away from the observer than the active edge is
		if (cur_vert->e != active_edge &&
			cur_vertex_edge_intersection.second > active_edge_intersection.second) {
			pq.insert(cur_vert->e);
		}
	}
}

vis_polygon = visibility_polygon(vis_poly);
*****************************************************************************/