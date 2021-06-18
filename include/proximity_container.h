#pragma once

#include "obstacle.h"

/**
 * A class to keep track of the distance between two objects.
 * This gets used inside the proximity queue.
 */
class proximity_container{
    public:
        /**
         * Default constructor.
         */
        proximity_container();

        /**
         * Constructor accepting both objects to be tracked.
         * @param o1 The first object. This must always be a user.
         * @param o2 The second object. This can be a user or any other object.
         */
        proximity_container(object* o1, object* o2);

        /**
         * Compute the shortest distance between the objects.
         */
        void update_distance();

        float get_distance() const;

        bool operator<(const proximity_container& c1);

        /**
         * Get o2, the obstacle whose distance from the user we are tracking.
         */
        object* get_obstacle();

        /**
         * Get the feature of o2 that is closest to o1. Here, a feature is
         * a segment (boundary wall) or a vertex.
         */
        std::vector<vec2f*> get_closest_feature();

    private:
        object* o1; // This is always a user.
        object* o2; // A user or environment obstacle.
        std::vector<vec2f*> closest_feature; // Vertices of feature of o2 that is closest to o1
        float distance;
};