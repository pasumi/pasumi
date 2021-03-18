#include "trans_view.h"


trans_view::trans_view() {

}

trans_view::trans_view(environment* e, vec2f _kernel) {
	kernel.x = _kernel.x;
	kernel.y = _kernel.y;
	vert_list = linked_list();
	int id_counter = 1;

	// Get all nodes and sort by angle
	std::vector<ll_node*> verts;
	for (vec2f* v : e->get_vertices()) {
		ll_node* n = new ll_node(kernel, *v, id_counter);
		verts.push_back(n);
		id_counter++;
	}
	for (auto o : e->get_obstacles()) {
		for (vec2f* v : o->get_vertices()) {
			ll_node* n = new ll_node(kernel, *v, id_counter);
			verts.push_back(n);
			id_counter++;
		}
	}
	struct {
		bool operator()(ll_node* node1, ll_node* node2) const {
			return node1->theta < node2->theta;
		}
	} sort_nodes;
	std::sort(verts.begin(), verts.end(), sort_nodes);

	// Add to the linked list
	for (ll_node* n : verts) {
		vert_list.add(n);
	}
}

trans_view::~trans_view() {

}

bool trans_view::update(vec2f new_kernel_pos, visibility_polygon* vis_poly) {
	ll_node* cur_node = vert_list.head;

	// Loop through all nodes
	while (cur_node->succ != vert_list.head) {
		float new_theta = atan2(cur_node->pos.y - new_kernel_pos.y,
								cur_node->pos.x - new_kernel_pos.x);
		new_theta = (new_theta > 0) ? new_theta : (math::two_pi + new_theta);
		cur_node->theta = new_theta;

		cur_node = cur_node->succ;
	}

	bool swap_happened = vert_list.sort(vis_poly);

	return swap_happened;
}