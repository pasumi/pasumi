#pragma once

#include "obstacle.h"

class proximity_container{
    public:
        proximity_container();
        proximity_container(object* o1, object* o2);
        void update_distance();
        float get_distance();
        float get_distance() const;
        bool operator<(const proximity_container& c1);
        object* get_obstacle();
        std::vector<vec2f*> get_closest_feature();

    private:
        object* o1; // A user
        object* o2; // A user or environment obstacle
        std::vector<vec2f*> closest_feature; // Vertices of feature of o2 that is closest to o1
        float distance;
};