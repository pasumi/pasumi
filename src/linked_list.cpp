#include <vector>
#include <algorithm>
#include <iostream>

#include "linked_list.h"
#include "timestep.h"

linked_list::linked_list() {
	head = nullptr;
	tail = nullptr;
	size = 0;	
}

linked_list::~linked_list() {

}

void linked_list::add(ll_node* n) {
	if (head == nullptr) {
		head = n;
		tail = n;
		head->pred = tail;
		head->succ = tail;
		tail->pred = head;
		tail->succ = head;
	}
	else {
		tail->succ = n;
		tail = n;
		tail->succ = head;
	}
	size++;
}

void linked_list::swap(ll_node* n1, ll_node* n2) {
	assert((n1->succ == n2 && n2->pred == n1) || (n2->succ == n1 && n1->pred == n2), 
			"Trying to swap nodes that are not adjacent in the list!");

	// n1 is before n2 in the list
	if (n1->succ == n2) {
		ll_node* node_preceeding_swap = n1->pred;
		ll_node* node_succeeding_swap = n2->succ;

		node_preceeding_swap->succ = n2;
		n2->succ = n1;
		n1->succ = node_succeeding_swap;

		n2->pred = node_preceeding_swap;
		n1->pred = n2;
		node_succeeding_swap->pred = n1;
	}
	// n2 is before n1 in the list
	else {
		ll_node* node_preceeding_swap = n2->pred;
		ll_node* node_succeeding_swap = n1->succ;

		node_preceeding_swap->succ = n1;
		n1->succ = n2;
		n2->succ = node_succeeding_swap;

		n1->pred = node_preceeding_swap;
		n2->pred = n1;
		node_succeeding_swap->pred = n2;
	}

}

bool linked_list::sort(visibility_polygon* vis_poly) {
	// TODO: improve this by doing insertion sort. can just get rid of the ll_node vector, and just rebuild the linked list via insertion sort. will save some time.
	std::vector<int> order_before;
	std::vector<ll_node*> list_copy;
	bool swap_happened = false;

	ll_node* cur = head;
	bool done = false;
	while (!done) {
		order_before.push_back(cur->id);
		list_copy.push_back(cur);
		cur = cur->succ;
		if (cur == head) {
			done = true;
		}
	}

	// Sort the list again
	struct {
		bool operator()(ll_node* node1, ll_node* node2) const {
			return node1->theta < node2->theta;
		}
	} sort_nodes;
	std::sort(list_copy.begin(), list_copy.end(), sort_nodes);

	// Rebuild list and compare new, sorted list for any swaps
	std::vector<int> swapped_ids;
	for (int i = 0; i < list_copy.size(); i++) {
		if (list_copy[i]->id != order_before[i]) {
			swapped_ids.push_back(list_copy[i]->id);
		}
	}

	if (timestep::num_timesteps == 139) {
		int t = 4;
	}
	for (int i : swapped_ids) {
		ll_node* n = get_node_by_id(i);
		if (vis_poly->is_vert_in(n->pos)) {
			swap_happened = true;
		}
	}

	// Rebuild the list
	head = nullptr;
	tail = nullptr;
	size = 0;
	for (int i = 0; i < list_copy.size(); i++) {
		this->add(list_copy[i]);
		if (list_copy[i]->id != order_before[i]) {
			swapped_ids.push_back(list_copy[i]->id);
		}
	}

	return swap_happened;
}

ll_node* linked_list::get_node_by_id(int id) {
	assert(id <= size);
	ll_node* cur = head;
	bool done = false;
	while (!done) {
		if (cur->id == id) {
			return cur;
		}
		cur = cur->succ;
		if (cur == head) {
			done = true;
		}
	}
	return nullptr;
}