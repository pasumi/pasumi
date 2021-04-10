#pragma once

#include "object.h"
#include "vec2f.h"
#include "environment.h"

class dynamic_obstacle : public object {
	public:
		/**
		 * Default constructor for dynamic obstacles
		 */
		dynamic_obstacle();

		/**
		 * Constructor for dynamic obstacles
		 */
		dynamic_obstacle(char* _path, bool is_phys);

		/**
		 * xxx
		 */
		~dynamic_obstacle();
		void update();
		std::vector<vec2f*> get_vertices();
		float distance(vec2f p);
		bool is_blocking_path(vec2f start, vec2f end, float radius);
		vec2f get_closest_wall(vec2f p);
		bool is_dynamic();

		vec2f* pos;
		environment* env;
		int path_index;
		int increment;
		std::vector<vec2f*> path;

	private:
		float radius = 2.0f;
};