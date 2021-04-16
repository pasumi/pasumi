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
		 * Destructor
		 */
		~dynamic_obstacle();

		/**
		 * Update the dynamic obstacle by one timestep.
		 */
		void update();

		/**
		 * Get a list of the vertices that define the obstacle.
		 * @return A vector of points defining the obstacle boundary. The 
				   vertices are stored in counter-clockwise order.
		 */
		std::vector<vec2f*> get_vertices();

		/**
		 * Compute the distance from the obstacle to point p.
		 * @param p The point p we wish to compute the distance to.
		 * @return the distance from p to the closest point on the obstacle.
		 */
		float distance(vec2f p);

		/**
		 * Determines if the obstacle is blocking (intersecting) a linear path.
		 * @param start The starting position on the path.
		 * @param end The ending position on the path.
		 * @param radius The radius of the user walking along the path.
		 * @return True if the person intersects with the obstacle along the path,
		 *		   false otherwise.
		 */
		bool is_blocking_path(vec2f start, vec2f end, float radius);

		/**
		 * Computes the edge of the obstacle that is closest to a point.
		 * @param p The point to which we wish to find the closest obstacle edge.
		 * @return The normalized vector (direction) of the obstacle edge that is
		 *		   closest to the point p.
		 */
		vec2f get_closest_wall(vec2f p);

		/**
		 * Check if the obstacle is dynamic or not.
		 * @return True if the obstacle is dynamic, false otherwise.
		 */
		bool is_dynamic();

		vec2f* pos; // Obstacle position
		environment* env; // The environment the obstacle is in.
		int path_index;
		int increment;
		std::vector<vec2f*> path;

	private:
		float radius = 2.0f;
};