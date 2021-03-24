#pragma once

#include <vector>

#include "vec2f.hpp"
#include "wall.h"
#include "obstacle.h"
#include "environment.h"

class physical_environment : public environment {
    public:
        physical_environment();
        physical_environment(std::vector<vec2f*> verts, std::vector<obstacle*> obstacles, vec2f center, char* name);
        physical_environment(std::string boundary_file, std::string obstacles_file, char* name);
        vec2f get_center();
        vec2f sample_point();
        void step();
        
        int num_vertices;
        float min_x;
        float max_x;
        float min_y;
        float max_y;

    private:
        vec2f center;
};