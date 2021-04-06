#pragma once

#include <vector>

#include "vec2f.h"
#include "object.h"

class wall : public object {
	public:
		/**
		 * Default wall constructor.
		 */
		wall();

		/**
		 * Wall constructor.
		 * @param p1 The first point of the wall.
		 * @param p2 The second point of the wall.
		 * @param is_phys If the wall is in the physical environment or not.
		 * @param is_static If the wall is a static obstacle or not.
		 */
		wall(vec2f* p1, vec2f* p2, bool is_phys, bool is_static);

		/**
		 * Default wall destructor.
		 */
		~wall();

		/**
		 * Return the vertices of the wall.
		 */
		std::vector<vec2f*> get_vertices();

		/**
		 * Return the distance between the wall and a point p.
		 */
		float distance(vec2f p);

		/**
		 * Check if the wall is blocking a path that a circle travels on.
		 * @param start The starting position of the circle.
		 * @param end The ending position of the circle.
		 * @param radius The radius of the circle.
		 * @return True if the circle intersects with the wall along its path,
		 *		   false otherwise.
		 */
		bool is_blocking_path(vec2f start, vec2f end, float radius);

		/**
		 * Return the normalized vector of the wall.
		 */
		vec2f get_closest_wall(vec2f p);

		/**
		 * Return if the wall is dynamic or not.
		 */
		bool is_dynamic();

		int num_vertices;

	private:
		vec2f* p1;
		vec2f* p2;
		std::vector<vec2f*> verts;
};