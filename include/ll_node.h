#pragma once

#include "vec2f.hpp"

class ll_node {
public:
	ll_node();
	ll_node(vec2f p);
	ll_node(vec2f kernel, vec2f p, int _id);
	~ll_node();

	ll_node* pred;
	ll_node* succ;
	vec2f pos;
	float theta;
	int id;

private:
};