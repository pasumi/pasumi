#pragma once

#include <vector>

#include "vec2f.h"
#include "wall.h"
#include "obstacle.h"
#include "environment.h"

/**
 * Class for physical environment objects.
 */
class physical_environment : public environment {
    public:
        /**
         * Default constructor.
         */
        physical_environment();

        /**
         * Construct a physical environment from an XML environment file.
         */
        physical_environment(fs::path env_file);

        /**
         * Advance the simulation for the physical environment by one timestep.
         */
        void step();

        vec2f get_center();

    private:
};