#pragma once

#include <set>

#include "node.h"
#include "edge.h"

class graph {
	public:
		graph();
		void add_node(node* n);
		void add_edge(node* n1, node* n2);

		std::set<node*> nodes; // Adjacency list
		std::set<edge*> edges;

	private:

};