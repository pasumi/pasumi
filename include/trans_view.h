#pragma once

#include "vec2f.hpp"
#include "geometry.h"
#include "environment.h"
#include "linked_list.h"
#include "visibility_polygon.h"

class trans_view {
public:
	trans_view();
	trans_view(environment* e, vec2f _kernel);
	~trans_view();
	bool update(vec2f new_kernel_pos, visibility_polygon* vis_poly);

private:
	vec2f kernel;
	linked_list vert_list;
};