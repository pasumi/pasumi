#include <iostream>
#include <assert.h>
#include <cassert>
#include <limits>

#include "proximity_container.h"
#include "math.hpp"
#include "wall.h"
#include "geometry.h"
#include "user.h"

proximity_container::proximity_container() {

}

proximity_container::proximity_container(object* o1, object* o2) {
    assert(o1->type == object::OBJECT_TYPE::USER);
	this->o1 = o1;
	this->o2 = o2;

    update_distance();
}

void proximity_container::update_distance() {
    std::vector<vec2f*> o1_verts = o1->get_vertices();
    std::vector<vec2f*> o2_verts = o2->get_vertices();

    // User-user distance computation
    if (o1->type == object::OBJECT_TYPE::USER && o2->type == object::OBJECT_TYPE::USER) {
        distance = geom::point_point_distance(o1_verts[0], o2_verts[0]);
        distance -= ((user*)o2)->radius; // Account for other user's radius. Normally, the user walks until their nose is almost touching the obstacle's closest feature. The closest feature for users is just their position, so without this line the user walks until their nose is touching the center of the other user (i.e. inside the other user) which is not allowed.
        closest_feature = o2->get_vertices();
    }
    // User-obstacle distance computation.
    else {
        float closest_distance = math::max_float;
        vec2f best_p1, best_p2;

        // Get distance from user to every line segment (wall) that comprises o2, record closest distance
        for (int i = 0; i < o2_verts.size(); i++) {
            vec2f* p1 = o2_verts[i % o2_verts.size()];
            vec2f* p2 = o2_verts[(i+1) % o2_verts.size()];
            float dist = geom::line_point_distance(p1, p2, o1_verts[0]);
            if (dist <= closest_distance) {
                vec2f* cur_pos = o1_verts[0];
                vec2f cur_heading = vec2f(rad_2_vec(((user*)o1)->state.get_phys_heading()));

                bool intersect_cur_closest = geom::ray_line_intersect(cur_pos, &cur_heading, &best_p1, &best_p2);
                bool intersect_new_closest = geom::ray_line_intersect(cur_pos, &cur_heading, p1, p2);

                if (intersect_new_closest) {
                    closest_distance = dist;
                    closest_feature.clear();
                    closest_feature.push_back(p1);
                    closest_feature.push_back(p2);
                }
            }
        }

        distance = closest_distance;
    }

    for (auto v : o1_verts) {
        delete v;
    }
}

std::vector<vec2f*> proximity_container::get_closest_feature() {
    return closest_feature;
}

float proximity_container::get_distance() {
    return distance;
}

float proximity_container::get_distance() const {
    return distance;
}

object* proximity_container::get_obstacle() {
    return o2;
}