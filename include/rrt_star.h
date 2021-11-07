#pragma once

#include "geometry.h"
#include "virtual_environment.hpp"
#include "vec2f.hpp"

struct rrt_node {
	rrt_node() {
		pos = vec2f(0, 0);
	}

	rrt_node(vec2f location) {
		pos = location;
		parent = nullptr;
	}

	void add_child(rrt_node* new_child) {
		children.push_back(new_child);
		new_child->parent = this;
	}

	void remove_child(rrt_node* child_to_remove) {
		for (int i = 0; i < children.size(); i++) {
			auto c = children[i];
			if (c->pos.x == child_to_remove->pos.x &&
				c->pos.y == child_to_remove->pos.y) {
				children.erase(children.begin() + i);
			}
		}
	}

	void update_parent(rrt_node* new_parent) {
		if (this->parent != nullptr) {
			this->parent->remove_child(this);
		}
		this->parent = new_parent;
		this->parent->add_child(this);
		this->parent->update_costs();
		update_costs();
	}

	void update_costs() {
		if (this->parent != nullptr) {
			this->parent_cost = length(this->pos - this->parent->pos);
			this->total_cost = this->parent->total_cost + this->parent_cost;
		}
		else {
			this->parent_cost = 0.0f;
			this->total_cost = this->parent_cost;
		}
	}

	~rrt_node() {

	}

	vec2f pos;
	rrt_node* parent;
	std::vector<rrt_node*> children;
	float parent_cost = 0.0f;
	float total_cost = 0.0f;
};

struct rrt_tree {
	rrt_tree() {

	}

	rrt_tree(rrt_node* r) {
		root = r;
		nodes.push_back(root);
	}

	~rrt_tree() {

	}

	void reset() {

	}

	void add_node(rrt_node* node_to_add, rrt_node* parent){
		parent->add_child(node_to_add);
		nodes.push_back(node_to_add);
	}

	std::vector<std::pair<rrt_node*, float>> get_n_nearest(vec2f sampled_pt, int n) {
		std::vector<std::pair<rrt_node*, float>> sorted_nodes;
		for (auto n : nodes) {
			sorted_nodes.push_back(std::make_pair(n, length(n->pos - sampled_pt)));
		}

		std::sort(sorted_nodes.begin(), sorted_nodes.end(), [](auto& left, auto& right) {
			return left.second < right.second;
			});

		std::vector<std::pair<rrt_node*, float>> n_nearest;
		for (int i = 0; i < math::min(n, sorted_nodes.size()); i++) {
			n_nearest.push_back(sorted_nodes[i]);
		}

		return n_nearest;
	}

	std::pair<rrt_node*, rrt_node*> connect_shortest_in_nearby(rrt_node* new_node, std::vector<std::pair<rrt_node*, float>> nearby_nodes, virtual_environment* venv) {
		for (int i = 0; i < nearby_nodes.size(); i++) {
			rrt_node* node = nearby_nodes[i].first;
			if (node->total_cost + length(node->pos - new_node->pos) < 
				best_cost && check_segment_for_collisions(node->pos, new_node->pos, venv)) {
				rrt_node* truncated_node = steer(node, new_node);
				return std::make_pair(truncated_node, node);
			}
		}
		return std::make_pair(root, root);
	}

	bool check_segment_for_collisions(vec2f p1, vec2f p2, virtual_environment* venv) {
		vec2f dir = (p2 - p1) / length(p2 - p1);
		vec2f n1 = vec2f(-dir.y, dir.x);
		vec2f n2 = vec2f(dir.y, -dir.x);
		vec2f s1_p1 = p1 + (n1 * 0.5f);
		vec2f s1_p2 = p2 + (n1 * 0.5f);
		vec2f s2_p1 = p1 + (n2 * 0.5f);
		vec2f s2_p2 = p2 + (n2 * 0.5f);
		bool s1_valid = venv->line_is_legal(&s1_p1, &s1_p2);
		bool s2_valid = venv->line_is_legal(&s2_p1, &s2_p2);
		bool orig_s_valid = venv->line_is_legal(&p1, &p2);
		return s1_valid && s2_valid && orig_s_valid;
	}

	rrt_node* steer(rrt_node* node_to_connect_to, rrt_node* node_to_steer_to) {
		vec2f dir = (node_to_steer_to->pos - node_to_connect_to->pos) / length(node_to_steer_to->pos - node_to_connect_to->pos);
		vec2f steered_pos = node_to_connect_to->pos + (dir * step_size);
		return new rrt_node(steered_pos);
	}

	void rewire(rrt_node* new_node, std::vector<std::pair<rrt_node*, float>> nearby_nodes) {
		for (int i = 0; i < nearby_nodes.size(); i++) {
			rrt_node* n = nearby_nodes[i].first;
			float potential_new_cost = new_node->total_cost + length(new_node->pos - n->pos);
			if (potential_new_cost < n->total_cost) {
				n->update_parent(new_node);
			}
		}
	}

	std::vector<vec2f> get_path(rrt_node* cur_node) {
		std::vector<vec2f> path;
		while (cur_node->parent != nullptr) {
			path.insert(path.begin(), cur_node->pos);
			cur_node = cur_node->parent;
		}
		path.insert(path.begin(), cur_node->pos);
		return path;
	}

	rrt_node* root;
	std::vector<rrt_node*> nodes;
	float best_cost = math::max_float;
	float step_size = 2.0f;
};

class rrt_star {

public:
	rrt_star();
	rrt_star(virtual_environment* venv);
	~rrt_star();
	std::vector<vec2f> check_solution(rrt_node* end_node);

	std::vector<vec2f> sample_path(vec2f start_point);
	rrt_tree tree;

private:
	int nearby_limit = 10;
	int num_iter = 1000;
	float step_size = 2.0f;
	virtual_environment* virt_env;
};