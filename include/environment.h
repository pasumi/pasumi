#pragma once

#include <vector>

class visibility_polygon;

#include "vec2f.h"
#include "obstacle.h"
#include "geometry.h"
#include "wall.h"
#include "math.hpp"
#include "ext/ghc/fs_std.hpp"
#include "ext/pugixml/pugixml.hpp"

class environment {
    public:
        /**
         * Default constructor for the environment.
         */
        environment();

        /**
         * Destructor for the environment.
         */
        ~environment();

        /**
         * Load the environment data from an XML file.
         * @param filepath The path to the XML file to be loaded.
         */
        void environment::load_xml_file(fs::path filepath);

        /**
         * Parse the border data from a <border> element in an XML environment
         * description file.
         * @param border_node The XML <border> element.
         */
        void environment::parse_border(pugi::xml_node border_node, bool is_phys);

        /**
         * Parse the vertex data from a <vertex> element in an XML environment
         * description file.
         * @param vert_string The text inside the <vertex> element.
         */
        vec2f environment::parse_vertex(char* vert_string);

        /**
         * Parse the obstacle data from an <obstacle> element in an XML environment
         * description file.
         * @param obs_node The XML <obstacle> element.
         * @param is_phys If the obstacle is a physical or virtual object.
         */
        obstacle* environment::parse_obstacle(pugi::xml_node obs_node, bool is_phys);

        /**
         * Get a list of vertices that define the boundary of the environment.
         * @return A vector containing the vertices of the boundary of the
         *         environment. The vertices are stored in counter-clockwise order.
         */
        std::vector<vec2f*> get_vertices();

        /**
         * Get a list of the boundary walls of the environment.
         * @return A vector containing the wall objects that define the boundary of
         *         the environment. The walls are stored in counter-clockwise order.
         */
        std::vector<wall*> get_walls();

        /**
         * Get a list of the obstacle objects that are in the environment.
         * @return A vector containing the obstacle objects in the environment.
         */
        std::vector<obstacle*> get_obstacles();

        /**
         * Determine if a point is in a valid position (no intersections/out of bounds)
         * in the environment.
         * @param p The point p whose validity we wish to check.
         * @return True if the point p is inside the environment boundaries and is
         *         not inside any environment obstacles, false otherwise.
         */
        bool point_is_legal(vec2f p);

        /**
         * Determine if a line segment is in a valid position (no intersections/out of
         * bounds) in the environment.
         * @param p1 The first point of the line segment.
         * @param p2 The second point of the line segment.
         * @return True if the segment does not intersect with the environment boundaries
         *         or any obstacles.
         */
        bool line_is_legal(vec2f* p1, vec2f* p2);

        /**
         * Get the distance between the point p and the closest obstacle.
         * Environment boundaries do count as obstacles.
         * @param p The point p from which we wish to find the distance to 
                    the closest obstacle.
         * @return The distance between p and the obstacle closest to p.
         */
        float get_closest_obstacle_distance(vec2f p);

        /**
         * Get the distance between the point p and the closest obstacle,
         * in a particular direction dir.
         * @param p The point p from which we wish to find the distance to 
                    the closest obstacle.
         * @param dir A unit vector denoting the direction in which we 
                      check for the closest obstacle to p.
         * @return The distance between p and the obstacle closest to p, in the
         *         direction dir.
         */
        float get_distance_in_direction_from_point(vec2f p, vec2f dir);

        /**
         * Update the environment to the next timestep.
         */
        virtual void step() = 0;

        /**
         * Sample a random valid point in the environment.
         * @return A random valid point in the environment.
         */
        virtual vec2f sample_point() = 0;

        std::string name;
        float min_x; // Minimum x-coordinate of the environment boundaries
        float max_x; // Maximum x-coordinate of the environment boundaries
        float min_y; // Minimum y-coordinate of the environment boundaries
        float max_y; // Maximum y-coordinate of the environment boundaries
        bool unbounded; // Flag if the environment has no boundaries.
        float area;

    protected:
        std::vector<vec2f*> verts; // Vertices of boundary in counter clockwise order
        std::vector<wall*> walls;
        std::vector<obstacle*> obstacles;
        vec2f center;
};