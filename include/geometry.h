#pragma once

#include <iostream>
#include <vector>
#include <random>

#include "math.hpp"
#include "vec2f.h"

namespace geom {
    /**
     * Orientation test.
     * Finds the orientation of ordered triplet of points (p1, p2, p3) 
     * by computing the determinant.
     * @param p1 The first point.
     * @param p2 The second point.
     * @param p3 The third point.
     * @return 0  if p, q, and r are colinear 
     *         -1 if p, q, and r are clockwise
     *         1  if p, q, and r are counterclockwise
     */
    inline int orient(vec2f* p1, vec2f* p2, vec2f* p3){
        return math::sign((p2->x - p1->x)*(p3->y - p1->y) - 
                          (p2->y - p1->y)*(p3->x - p1->x));
    }

    /**
     * Check if two lines intersect.
     * @param s1_1 The first point of the first line segment.
     * @param s1_2 The second point of the first line segment.
     * @param s2_1 The first point of the second line segment.
     * @param s2_2 The second point of the second line segment.
     * @return True if s1 intersects s2, false otherwise.
     */
    inline bool line_line_intersect(vec2f* s1_1, vec2f* s1_2, vec2f* s2_1, vec2f* s2_2){
        return ((orient(s1_1, s1_2, s2_1) != orient(s1_1, s1_2, s2_2)) && 
                (orient(s2_1, s2_2, s1_1) != orient(s2_1, s2_2, s1_2)));
    }

    /**
     * Compute the distance between two points.
     * @param p1 The first point.
     * @param p2 The second point.
     * @return The distance between p1 and p2.
     */
    inline float point_point_distance(vec2f* p1, vec2f* p2) {
        return distance(*p1, *p2);
    }

    /**
     * Compute the minimum distance between a point and a line. 
     * @param s1_1 The first point of the first line segment.
     * @param s1_2 The second point of the first line segment.
     * @param p1 The free point.
     * @return The length of the projection of the point onto the line.
     */
    static float line_point_distance(vec2f* s1, vec2f* s2, vec2f* p1) {
        float t = dot(*p1 - *s1, *s2 - *s1) / (length(*s2 - *s1) * length(*s2 - *s1));
        t = math::min(math::max(t, 0), 1);
        vec2f closest_point = *s1 + ((*s2 - *s1) * t);
        return length(closest_point - *p1);
    }

    /**
     * Compute the time at which two circles intersect, given each circle's 
     * starting and ending position. 
     * Via https://www.gamasutra.com/view/feature/3383/simple_intersection_tests_for_games.php?page=2
     * @param c1_1 The first position of the first circle.
     * @param c1_2 The second position of the first circle.
     * @param c2_1 The first position of the second circle.
     * @param c2_2 The second position of the second circle.
     * @param radius1 The radius of the first circle.
     * @param radius1 The radius of the second circle.
     * @return The time of intersection between c1 and c2, or -1 if they do not intersect.
     */
    static float circle_circle_intersect(vec2f c1_1, vec2f c1_2, vec2f c2_1, vec2f c2_2, float radius1, float radius2) {
        vec2f v_a = c1_2 - c1_1;
        vec2f v_b = c2_2 - c2_1;
        vec2f AB = c2_1 - c1_1;
        vec2f v_ab = v_b - v_a;
        float r_ab = radius1 + radius2;
        float a = dot(v_ab, v_ab);
        float b = 2 * dot(v_ab, AB);
        float c = dot(AB, AB) - (r_ab * r_ab);
        float u0, u1;
        if (math::quadratic(a, b, c, u0, u1)) return math::min(u0, u1);
        return -1.0f;
    }

    /**
     * Compute the intersection between a ray and a line segment.
     * Via https://stackoverflow.com/questions/53893292/how-to-calculate-ray-line-segment-intersection-preferably-in-opencv-and-get-its
     * @param ray_origin The oigin point of the ray.
     * @param ray_dir The direction of the ray.
     * @param s1 The first point of the line segment.
     * @param s2 The second point of the line segment.
     * @return The time (length) along the ray at which the ray intersects the
     *         line segment, or -1 if they do not intersect.
     */
    static float ray_line_intersect(vec2f* ray_origin, vec2f* ray_dir, vec2f* s1, vec2f* s2) {
        vec2f p1 = *ray_origin - *s1;
        vec2f p2 = *s2 - *s1;
        vec2f p3 = vec2f(-ray_dir->y, ray_dir->x);

        float d = dot(p2, p3);
        if (abs(d) < math::epsilon)
            return -1.0f;

        float t1 = cross(p2, p1) / d;
        float t2 = dot(p1, p3) / d;

        if (t1 >= 0.0f && (t2 >= 0.0f && t2 <= 1.0))
            return t1; // t1 is the intersection point

        return -1.0f;
    }

    /**
     * Computes if the difference between two values is within epsilon.
     * @param v1 The first value.
     * @param v2 The second value.
     * @return True if the distance between v1 and v2 is less than or equal to
     *         epsilon, false otherwise.
     */
    static bool approx_equal(float v1, float v2) {
        return (abs(v1 - v2) <= math::epsilon);
    }

    /**
     * Compute the time at which a circle intersects a line segment.
     * @param s1 The first point of the line segment.
     * @param s2 The second point of the line segment.
     * @param circle_p1 The first position of the circle.
     * @param circle_p2 The second position of the circle.
     * @param circle_radius The radius of the circle.
     * @return The time at which the circle intersects the line segment, or
     *         -1 if the circle does not intersect the line segment.
     */
    static float line_circle_intersect(vec2f* s1, vec2f* s2, vec2f* circle_p1, vec2f* circle_p2, float circle_radius) {
        vec2f offset = normalize(*s2 - *s1) * circle_radius;
        vec2f up = vec2f(-offset.y, offset.x);
        vec2f down = vec2f(offset.y, -offset.x);
        vec2f a = normalize(*s2 - *s1);
        s1 = new vec2f(*s1 + normalize(*s1 - *s2) * 10.0f);
        s2 = new vec2f(*s2 + normalize(*s2 - *s1) * 10.0f);
        vec2f* s1_1 = new vec2f(*s1 + up);
        vec2f* s1_2 = new vec2f(*s2 + up);
        vec2f* s2_1 = new vec2f(*s1 + down);
        vec2f* s2_2 = new vec2f(*s2 + down);

        float t1 = -1.0f;
        float t2 = -1.0f;
        if (line_line_intersect(s1_1, s1_2, circle_p1, circle_p2)) {
            vec2f v = normalize(*circle_p2 - *circle_p1);
            t1 = ray_line_intersect(circle_p1, &v, s1_1, s1_2);
        }
        if (line_line_intersect(s2_1, s2_2, circle_p1, circle_p2)) {
            vec2f v = normalize(*circle_p2 - *circle_p1);
            t2 = ray_line_intersect(circle_p1, &v, s2_1, s2_2);
        }

        if (t1 >= 0.0f && t2 >= 0.0f) return math::min(t1, t2);
        if (t1 >= 0.0f) return t1;
        return t2;
    }

    /**
     * Ray-shooting algorithm to check if a point is in a polygon or not. 
     * Via https://www.ics.uci.edu/~eppstein/161/960307.html
     * @param p The point to query.
     * @param verts The vertices of the polygon boundary, in CCW order.
     * @return True if p is strictly in the polygon defined by verts, 
               false otherwise (outside or on the boundary).
     */
    static bool point_in_polygon(vec2f p, std::vector<vec2f*> verts) {
        int crossings = 0;
        for (int i = 0; i < verts.size(); i++) {
            vec2f* cur = verts[i];
            vec2f* prev = (i == 0) ? verts[verts.size() - 1] : verts[i - 1];
            vec2f* next = verts[(i + 1) % verts.size()];

            if (cur->x < p.x && p.x < next->x ||
                cur->x > p.x && p.x > next->x) {
                float t = (p.x - next->x) / (cur->x - next->x);
                float cy = t * cur->y + (1 - t) * next->y;
                if (p.y == cy) return false; // On boundary
                else if (p.y > cy) crossings++;
            }
            if (cur->x == p.x && cur->y <= p.y) {
                if (cur->y == p.y) return false; // On boundary
                if (next->x == p.x) {
                    if (cur->y <= p.y && p.y <= next->y ||
                        cur->y >= p.y && p.y >= next->y)
                        return false; // On boundary
                }
                else if (next->x > p.x) crossings++;
                if (prev->x > p.x) crossings++;
            }
        }
        if (crossings % 2 == 1) return true; // Inside
        else return false; // Outside
    }

    /**
     * Compute the normal of the line segment (s1, s2) that is on the same
     * same side of (s1, s2) that p is on. In other words, it computes the normal
     * that "points" to p.
     * @param s1 The first point of the line segment.
     * @param s2 The second point of the line segment.
     * @param p The free point.
     * @return The normal vector of the line segment that lies on the 
               same side of the line segment as p does.
     */
    static vec2f normal_to_point(vec2f* s1, vec2f* s2, vec2f* p) {
        vec2f line_dir = normalize(*s2 - *s1);
        vec2f n1 = vec2f(-line_dir.y, line_dir.x);
        vec2f n2 = vec2f(line_dir.y, -line_dir.x);
        vec2f line_to_point = normalize(*p - *s1);

        if (dot(line_to_point, n1) > 0.0f) {
            return n1;
        }
        else {
            return n2;
        }
    }

    /**
     * Compute the area of a polygon using the shoelace formula.
     * @param pts The points of the polygon, in CCW order.
     * @return The area of the polygon defined by pts.
     */
    static float polygon_area(std::vector<vec2f*> pts) {
        float area = 0.0f;
        vec2f* p1 = pts[0];
        vec2f* pn = pts[pts.size() - 1];

        float sum1 = 0.0f;
        float sum2 = 0.0f;
        for (int i = 0; i < pts.size() - 1; i++) {
            vec2f* cur = pts[i];
            vec2f* next = pts[i + 1];
            sum1 += (cur->x * next->y);
            sum2 += (next->x * cur->y);
        }
        sum1 += pn->x * p1->y;
        sum2 += p1->x * pn->y;

        return (sum1 - sum2) * 0.5f;
    }

    /**
     * Compute the perimeter of a polygon.
     * @param pts The points of the polygon, in CCW order.
     * @return The perimeter of the polygon defined by pts.
     */
    static float polygon_perimeter(std::vector<vec2f*> pts) {
        float perim = 0.0f;

        for (int i = 0; i < pts.size(); i++) {
            vec2f* p1 = pts[i];
            vec2f* p2 = pts[(i+1)%pts.size()];
            perim += length(*p1 - *p2);
        }

        return perim;
    }

    /**
     * Uniformly sample a random point inside a triangle.
     * @param p1 The first point of the triangle.
     * @param p2 The second point of the triangle.
     * @param p3 The third point of the triangle.
     * @return A random point inside the triangle defined by p1, p2, and p3.
     */
    static vec2f sample_triangle(vec2f* p1, vec2f* p2, vec2f* p3) {
        std::uniform_real_distribution<float> dis(0, 1);
        std::random_device rd;
        std::mt19937 gen(rd());
        float r1 = dis(gen);
        float r2 = dis(gen);
        return ((1.0f - math::sqrt(r1)) * (*p1)) + 
                (math::sqrt(r1) * (1.0f - r2) * (*p2)) + 
                ((r2 * math::sqrt(r1)) * (*p3));
    }
}
