#pragma once

#include <vector>

#include "vec2f.h"
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

    private:
};