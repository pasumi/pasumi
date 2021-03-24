#pragma once

#include <vector>

class visibility_polygon;

#include "vec2f.h"
#include "obstacle.h"
#include "geometry.h"
#include "wall.h"

class environment {
    public:
        environment();
        ~environment();
        std::vector<vec2f*> get_vertices();
        std::vector<wall*> get_walls();
        std::vector<obstacle*> get_obstacles();
        bool point_is_legal(vec2f p);
        bool line_is_legal(vec2f* p1, vec2f* p2);
        float get_closest_obstacle_distance(vec2f pt);
        float get_distance_in_direction_from_point(vec2f p, vec2f dir);
        virtual void step() = 0;
        virtual vec2f sample_point() = 0;

        char* name;
        float min_x;
        float max_x;
        float min_y;
        float max_y;
        bool unbounded;
        float area;

    protected:
        std::vector<vec2f*> verts; // Vertices of boundary in counter clockwise order
        std::vector<wall*> walls;
        std::vector<obstacle*> obstacles;
};