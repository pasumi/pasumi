#include "ll_node.h"

ll_node::ll_node() {

}

ll_node::ll_node(vec2f p) {
	pos.x = p.x;
	pos.y = p.y;
}

ll_node::ll_node(vec2f kernel, vec2f p, int _id) {
	pos.x = p.x;
	pos.y = p.y;
	id = _id;

	float temp = atan2(p.y - kernel.y, p.x - kernel.x);
	theta = temp > 0 ? temp : (math::two_pi + temp);
}

ll_node::~ll_node() {

}
