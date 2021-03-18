#include <cassert>

#include "graph.h"

graph::graph() {

}

void graph::add_node(node* n) {
	assert(nodes.insert(n).second);
}

void graph::add_edge(node* n1, node* n2) {
	// Ensure the nodes exist in the graph
	assert(nodes.find(n1) != nodes.end());
	assert(nodes.find(n2) != nodes.end());

	n1->add_neigbor(n2);
	n2->add_neigbor(n1);

	edge* e = new edge(n1, n2);
	n1->add_edge(e);
	n2->add_edge(e);
	this->edges.insert(e);
}