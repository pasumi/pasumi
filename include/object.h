#pragma once

#include <vector>

#include "vec2f.h"

class object {
	public:
		enum class OBJECT_TYPE { USER, WALL, OBSTACLE };
		enum class SPACE_TYPE { PHYS, VIRT };
		enum class MOVEMENT_TYPE { STATIC, DYNAMIC };

		virtual std::vector<vec2f*> get_vertices() = 0;
		virtual float distance(vec2f p) = 0; // Distance from object to p
		virtual bool is_blocking_path(vec2f start, vec2f end, float radius) = 0;
		virtual vec2f get_closest_wall(vec2f p) = 0;
		virtual bool is_dynamic() = 0;

		int num_vertices;
		OBJECT_TYPE type;
		SPACE_TYPE space;
		MOVEMENT_TYPE mvmt;

	private:
};