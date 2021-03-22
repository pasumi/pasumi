#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "vec2f.hpp"
#include "wall.h"
#include "object.h"

class obstacle : public object {
	public:
		obstacle();
		obstacle(std::vector<vec2f*> verts, bool is_phys, bool is_static);
		obstacle(char* _path, bool is_phys, bool is_static, float _radius);
		obstacle(bool is_phys, bool is_static, float _radius);
		~obstacle();
		std::vector<vec2f*> get_vertices();
		std::vector<wall*> get_walls();
		float distance(vec2f p);
		bool is_blocking_path(vec2f start, vec2f end, float radius);
		vec2f get_closest_wall(vec2f p);
		int vertex_buffer_size();
		int index_buffer_size();
		void step();

		vec2f sample_point_inside();
		void wait_for_reset(int wait_timer);
		bool is_dynamic();
		void reset_state();
		void add_path(std::vector<vec2f*> new_path);
		void set_pos(vec2f p);

		int num_vertices;
		std::vector<float> gl_verts;
		std::vector<unsigned int> gl_indices;

		vec2f* pos;
		//environment* env;
		int path_index;
		int increment;
		std::vector<vec2f*> path;
		float radius;
		std::vector<vec2f*> base_verts;


		std::vector<std::vector<vec2f*>> rvo_paths;
		int cur_path;

	private:
		std::vector<vec2f*> verts;
		std::vector<wall*> walls;

		int RESET_WAIT_LIMIT = 30;
		int post_reset_timer;
};