#pragma once

#include <vector>

#include "vec2f.h"
#include "wall.h"
#include "obstacle.h"
#include "environment.h"

class virtual_environment : public environment {
    public:
        virtual_environment();
        virtual_environment(fs::path env_file);
        vec2f sample_point();
        void step();

    private:
};