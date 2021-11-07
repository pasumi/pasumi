#include "rrt_star.h"

rrt_star::rrt_star() {

}

rrt_star::rrt_star(virtual_environment* venv) {
	virt_env = venv;
}

rrt_star::~rrt_star() {

}

std::vector<vec2f> rrt_star::sample_path(vec2f start_point) {
	vec2f start_pos = start_point;
	rrt_node* start_node = new rrt_node(start_pos);
	vec2f end_pos = virt_env->sample_point();
	rrt_node* end_node = new rrt_node(end_pos);
	tree = rrt_tree(start_node);
	bool found_path = false;

	while (!found_path) {
		for (int i = 0; i < num_iter; i++) {
			vec2f sampled_point = virt_env->sample_point();
			std::vector<std::pair<rrt_node*, float>> nearby_verts = tree.get_n_nearest(sampled_point, nearby_limit);
			std::pair<rrt_node*, rrt_node*> connect_result = tree.connect_shortest_in_nearby(new rrt_node(sampled_point), nearby_verts, virt_env);

			if (connect_result.first->pos.x != tree.root->pos.x &&
				connect_result.first->pos.y != tree.root->pos.y) {
				rrt_node* steered_node_to_add = connect_result.first;
				rrt_node* node_to_connect_to = connect_result.second;

				if (virt_env->point_is_legal(steered_node_to_add->pos) &&
					tree.check_segment_for_collisions(steered_node_to_add->pos, node_to_connect_to->pos, virt_env)) {
					tree.add_node(steered_node_to_add, node_to_connect_to);
					tree.rewire(steered_node_to_add, nearby_verts);
				}
			}

			std::vector<vec2f> potential_solution = check_solution(end_node);
			if (potential_solution.size() > 0) {
				potential_solution = check_solution(end_node);
				/*std::cout << "------------" << std::endl;
				for (auto n : potential_solution) {
					std::cout << n << std::endl;
				}*/
				return potential_solution;
			}
		}

		end_pos = virt_env->sample_point();
		end_node = new rrt_node(end_pos);
		tree = rrt_tree(start_node);
		std::cout << "failed to find RRT path! Trying again..." << std::endl;
		std::cout << start_point << std::endl;
		std::cout << end_pos << std::endl;
	}	

	std::cout << "failed to find path!" << std::endl;
}

std::vector<vec2f> rrt_star::check_solution(rrt_node* end_node) {
	std::vector<vec2f> failed_soln;
	std::vector<std::pair<rrt_node*, float>> nearest = tree.get_n_nearest(end_node->pos, 1);
	if (nearest.size() == 0) {
		return failed_soln;
	}

	rrt_node* nearest_to_goal = tree.get_n_nearest(end_node->pos, 1)[0].first;

	if (nearest_to_goal->pos.x == end_node->pos.x &&
		nearest_to_goal->pos.y == end_node->pos.y) {
		return tree.get_path(end_node);
	}

	if (length(nearest_to_goal->pos - end_node->pos) > tree.step_size) {
		return failed_soln;
	}

	if (tree.check_segment_for_collisions(nearest_to_goal->pos, end_node->pos, virt_env)) {
		tree.add_node(end_node, nearest_to_goal);
		return tree.get_path(end_node);
	}
	else {
		return failed_soln;
	}
}