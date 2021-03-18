#pragma once

#include <set>

#include "vec2f.hpp"
#include "environment.h"
#include "visibility_polygon.h"

class edge;

class node {
	public:
		node(vec2f location);
		bool add_neigbor(node* n);
		bool add_edge(edge* e);
		void compute_visibility_polygon(environment* env);
		void update_features(std::set<node*> all_nodes);
		polygon_similarity compute_similarity(node* other_node, float phys_heading, float virt_heading);
		float compute_distance_loss(node* other_node, float phys_heading, float virt_heading);

		vec2f location;
		std::set<edge*> edges;
		std::set<node*> neigbors;
		visibility_polygon vis_polygon;

	private:
		int get_shortest_path_length(node* source, node* destination, std::set<node*> all_nodes);
		void compute_nearest_features(vec2f heading);
		/*
		void process_edge(wall* w, std::vector<vis_poly_edge*>& edges, std::vector<vis_poly_vertex*>& polar_pts);
		void handle_event_point(std::vector<vis_poly_vertex*>& polar_pts, int i, std::vector<vis_poly_vertex*>& vis_poly, vis_poly_edge* active_edge, std::priority_queue<vis_poly_vertex*>& event_queue);
		vis_poly_edge* get_active_edge(vis_poly_vertex* cur_vert, std::vector<vis_poly_edge*> edges);
		vis_poly_edge* get_next_edge(int cur_index, std::vector<vis_poly_vertex*> verts);
		std::vector<std::pair<vis_poly_edge*, float>> get_intersecting_edges(vis_poly_vertex* cur_vert, std::vector<vis_poly_edge*> edges);
		void extract_boundary_points_and_edges(std::vector<vis_poly_vertex*> verts, std::vector<vis_poly_vertex*>& polar_pts, std::vector<vis_poly_edge*>& edges);
		void extract_obstacle_points_and_edges(std::vector<vis_poly_vertex*> verts, std::vector<vis_poly_vertex*>& polar_pts, std::vector<vis_poly_edge*>& edges);
		bool check_if_visible(vis_poly_edge* edge_to_check, vis_poly_edge* active_edge);
		*/

		int neighborhood_size;
		float clustering_coeff;
		float mean_shortest_path_length;
		bool visited = false; // for BFS
		float distance_north;
		float distance_east;
		float distance_south;
		float distance_west;
};