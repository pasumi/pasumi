#pragma once

#include <vector>

#include "vec2f.h"

/**
 * A base class for all objects in physcial and virtual environments.
 * Generally, this class will not be directly used and instead we use it as the
 * parent class from which we inherit when defining our other object classes (such
 * as obstacles, walls, and users).
 */
class object {
	public:
		enum class OBJECT_TYPE { USER, WALL, OBSTACLE };
		enum class SPACE_TYPE { PHYS, VIRT };
		enum class MOVEMENT_TYPE { STATIC, DYNAMIC };

		virtual std::vector<vec2f*> get_vertices() = 0;

		/**
		 * Computes and returns the distance between the object and a given point.
		 * @param p The point from which we wish to measure the distance to this object.
		 * @return The closest distance between this object and the point p.
		 */
		virtual float distance(vec2f p) = 0;

		/**
		 * Determines if the object is blocking the path on a straight line between a 
		 * given start and end point.
		 * @param start The starting point of the path.
		 * @param end The end point of the path.
		 * @param radius The radius of the agent that is travelling along the path.
		 * @return True if the object blocks the agent on its path, false otherwise.
		 */
		virtual bool is_blocking_path(vec2f start, vec2f end, float radius) = 0;

		/**
		 * Gets the wall of this object that is closest to the given point.
		 * @param p The point from which we wish to find the closest wall of the object.
		 * @return The wall object that is closest to the given point p.
		 */
		virtual vec2f get_closest_wall(vec2f p) = 0;

		/**
		 * Determine if this object is dynamic (moving) or not.
		 * @return True of the object moves in its environment, false otherwise.
		 */
		virtual bool is_dynamic() = 0;

		int num_vertices;
		OBJECT_TYPE type;
		SPACE_TYPE space;
		MOVEMENT_TYPE mvmt;

	private:
};