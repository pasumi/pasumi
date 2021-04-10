#pragma once

#include <ostream>
#include <cassert>

#include "math.hpp"

class vec2f{
    public:
        float x;
        float y;

        inline vec2f() { this->x = 0; this->y = 0; }
        inline vec2f(float x, float y) { this->x = x; this->y = y; }
        inline vec2f(const vec2f& v2) { x = v2.x; y = v2.y; }
        inline ~vec2f() {};

        // Accessor operations
        inline float& operator[](int i) { return (&x)[i]; }
		inline const float& operator[](int i) const { return (&x)[i]; }

        friend std::ostream& operator<<(std::ostream& os, vec2f const& vec) {
            return os << "(" << vec.x << ", " << vec.y << ")";
        }
};

// Math operations
inline vec2f operator+(const vec2f& a, const vec2f& b) { return vec2f(a.x + b.x, a.y + b.y); }
inline vec2f operator-(const vec2f& a, const vec2f& b) { return vec2f(a.x - b.x, a.y - b.y); }
inline vec2f operator*(const vec2f& a, const vec2f& b) { return vec2f(a.x * b.x, a.y * b.y); }
inline vec2f operator*(const vec2f& a, const float b)  { return vec2f(a.x * b, a.y * b); }
inline vec2f operator*(const float a, const vec2f& b)  { return vec2f(a * b.x, a * b.y); }
inline vec2f operator/(const vec2f& a, const vec2f& b) { return vec2f(a.x / b.x, a.y / b.y); }
inline vec2f operator/(const vec2f& a, const float b)  { return vec2f(a.x / b, a.y / b); }

// Assignment operations
inline vec2f& operator+=(vec2f& a, const vec2f& b) { return a = a + b; }
inline vec2f& operator-=(vec2f& a, const vec2f& b) { return a = a - b; }
inline vec2f& operator*=(vec2f& a, const vec2f& b) { return a = a * b; }
inline vec2f& operator/=(vec2f& a, const vec2f& b) { return a = a / b; }

// Comparison operators
inline bool operator==(const vec2f& a, const vec2f& b) { 
    return (math::abs(a.x - b.x) <= math::epsilon && 
            math::abs(a.y - b.y) <= math::epsilon);
    //return (a.x == b.x && a.y == b.y);
}
inline bool operator!=(const vec2f& a, const vec2f& b) { return (a.x != b.x || a.y != b.y); }

inline float dot(const vec2f& a, const vec2f& b)    { return a.x * b.x + a.y * b.y; }
inline float cross(const vec2f& a, const vec2f& b)  { return a.x * b.y - a.y * b.x; }

inline float length(const vec2f& a) { return math::sqrt(dot(a, a)); }
inline vec2f normalize(const vec2f& a) {
    auto l = length(a);
    return (l != 0) ? a / l : a;
}
inline float distance(const vec2f& a, const vec2f& b) { return length(a - b); }

static float angle(const vec2f& v1, const vec2f& v2){
    float a = length(v1);
    float b = length(v2);
    assert(math::abs(length(v1) - 1) < math::epsilon && 
           math::abs(length(v2) - 1) < math::epsilon);
    float denom = math::sqrt(length(v1) * length(v2));
    if (denom < math::epsilon) return 0;
    else{
        float dot_prod = math::clamp(dot(v1, v2) / denom, -1, 1);
        return math::acos(dot_prod);
    }
}

/**
 * Signed angle between vectors v1 and v2. If the direction from v1 to v2 is
 * clockwise (turn to the right), the angle is negative. If the direction from
 * v1 to v2 is counterclockwise (turn to the left), the angle is positive.
 * Angle returned is in radians.
 */
static float signed_angle(const vec2f& v1, const vec2f& v2){
    float unsigned_angle = angle(v1, v2);
    float cross_x = v1[1] * 0 - 0 * v2[1];
    float cross_y = 0 * v2[0] - v1[0] * 0;
    float cross_z = v1[0] * v2[1] - v1[1] * v2[0];
    int sign = math::sign(0 * cross_x + 0 * cross_y + 1 * cross_z);
    return unsigned_angle * sign;
}

/**
 * Convert an angle to a unit vector.
 */
inline vec2f rad_2_vec(const float& rad) {
    return vec2f(cos(rad), sin(rad));
}

/**
 * Convert a unit vector to an angle.
 */
inline float vec_2_rad(const vec2f& v) {
    float angle = math::atan2(v.y, v.x);
    if (angle < 0.0f) angle += math::two_pi;
    return angle;
}

inline vec2f polar(const vec2f& origin, const vec2f& to_convert) {
    float r = length(origin - to_convert);
    float theta = signed_angle(vec2f(1, 0), normalize(to_convert - origin));
    if (theta < 0) {
        theta = math::two_pi - (math::abs(theta));
    }
    return vec2f(r, theta);
}

/**
 * Rotate a vector around a point. If theta is positive, the vector is rotated to the
 * left; if theta is negative, the vector is rotated to the right (from the perspective
 * of you standing at origin looking in the direction of to_rotate).
 */
inline vec2f rotate_around(const vec2f& origin, const vec2f& to_rotate, const float& theta) {
    vec2f centered_point = to_rotate - origin;
    vec2f rotated;
    rotated.x = (math::cos(theta) * centered_point.x) - (math::sin(theta) * centered_point.y);
    rotated.y = (math::sin(theta) * centered_point.x) + (math::cos(theta) * centered_point.y);
    rotated += origin;
    return rotated;
}