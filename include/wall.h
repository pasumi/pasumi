#pragma once

#include <vector>

#include "vec2f.h"
#include "object.h"

class wall : public object {
	public:
		wall();
		~wall();
		wall(vec2f* p1, vec2f* p2, bool is_phys, bool is_static);
		std::vector<vec2f*> get_vertices();
		float distance(vec2f p);
		bool is_blocking_path(vec2f start, vec2f end, float radius);
		vec2f get_closest_wall(vec2f p);
		int vertex_buffer_size();
		int index_buffer_size();
		bool is_dynamic();

		int num_vertices;

	private:
		vec2f* p1;
		vec2f* p2;
		std::vector<vec2f*> verts;
};