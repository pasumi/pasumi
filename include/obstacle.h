#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "vec2f.h"
#include "wall.h"
#include "object.h"

/**
 * A class for environment obstacles.
 */
class obstacle : public object {
	public:
		/**
		 * Default constructor.
		 */
		obstacle();

		/**
		 * Constructor for arbitrary polygonal obstacles. 
		 */
		obstacle(std::vector<vec2f*> verts, bool is_phys, bool is_static);

		/**
		 * Constructor for dynamic circular obstacles.
		 */
		obstacle(char* _path, bool is_phys, bool is_static, float _radius);

		/**
		 * Constructor for static circular obstacles.
		 */
		obstacle(bool is_phys, bool is_static, float _radius);

		~obstacle();

		/**
		 * Return a vector of the obstacle boundary vertices in CCW order.
		 */
		std::vector<vec2f*> get_vertices();

		/**
		 * Return a vector of the obstacle boundary walls.
		 */
		std::vector<wall*> get_walls();

		/**
		 * Compute the shortest distance between a point and the obstacle.
		 * @param p The point to which we compute the distance from the obstacle.
		 */
		float distance(vec2f p);

		/**
		 * Determine if the obstacle is blocking a circle that follows a 
		 * straight-line segment path.
		 * @param start The start point of the path.
		 * @param end The end point of the path.
		 * @param radius The radius of the circle that travels along the path.
		 */
		bool is_blocking_path(vec2f start, vec2f end, float radius);

		/**
		 * Get the wall of the obstacle that is closest to a given point.
		 * @param p The point to which we wish to find the clsoest obstacle wall.
		 */
		vec2f get_closest_wall(vec2f p);


		/**
		 * Advance the obstacle's simulation by one step. Only useful for non-static
		 * obstacles.
		 */
		void step();

		/**
		 * Prevent the obstacle from moving until the user has moved far enough awya.
		 * @param wait_timer The number of timesteps that the obstacle should pause 
		 *                   for.
		 */
		void wait_for_reset(int wait_timer);

		bool is_dynamic();

		/**
		 * Reset the obstacle's state back to its initial one.
		 */
		void reset_state();

		void add_path(std::vector<vec2f*> new_path);

		void set_pos(vec2f p);

		// OpenGL stuff
		int vertex_buffer_size();
		int index_buffer_size();

		vec2f* pos; // Position of the obstacle's centroid
		int path_index;
		int increment;
		std::vector<vec2f*> path;
		float radius;
		std::vector<vec2f*> base_verts;
		std::vector<std::vector<vec2f*>> rvo_paths;
		int cur_path;

		// OpenGL stuff
		int num_vertices;
		std::vector<float> gl_verts;
		std::vector<unsigned int> gl_indices;

	private:
		std::vector<vec2f*> verts;
		std::vector<wall*> walls;

		int RESET_WAIT_LIMIT = 30;
		int post_reset_timer;
};