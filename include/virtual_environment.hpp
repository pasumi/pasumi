#pragma once

#include <vector>

#include "vec2f.hpp"
#include "wall.h"
#include "obstacle.h"
#include "environment.h"

class virtual_environment : public environment {
    public:
        virtual_environment();
        virtual_environment(std::vector<vec2f*> verts, char* name);
        virtual_environment(std::string boundary_file, std::string obstacles_file, char* name);
        vec2f sample_point();
        void step();
        /*
        std::vector<vec2f> get_vertices();
        std::vector<wall*> get_walls();
        std::vector<obstacle*> get_obstacles();
        bool point_is_legal(vec2f p);
        bool line_is_legal(vec2f* p1, vec2f* p2);

        float min_x;
        float max_x;
        float min_y;
        float max_y;
        bool unbounded;
        */
    private:
        void setup_RVO_obstacles();
        /*
        std::vector<vec2f> verts; // Vertices of boundary in counter clockwise order
        std::vector<wall*> walls;
        std::vector<obstacle*> obstacles;
        */
};