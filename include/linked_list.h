#pragma once

#include "ll_node.h"
#include "visibility_polygon.h"

class linked_list {
public:
	linked_list();
	~linked_list();
	void add(ll_node* n);
	void swap(ll_node* n1, ll_node* n2);
	bool sort(visibility_polygon* vis_poly);
	ll_node* get_node_by_id(int id);

	ll_node* head;
	ll_node* tail;
	int size;

private:
};