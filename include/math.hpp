#pragma once

#include <cmath>

namespace math{
	const float pi = 3.14159265358979323846;
	const float two_pi = 6.28318530717958647692;
    const float epsilon = 0.0001;
	const float min_float = std::numeric_limits<float>::min();
	const float max_float = std::numeric_limits<float>::max();

	inline int abs(int a) { return a < 0 ? -a : a; }
	inline float abs(float a) { return a < 0 ? -a : a; }
    inline int sign(float val) { return (0 < val) - (val < 0); }
    inline float min(float a, float b) { return (a < b) ? a : b; }
    inline float max(float a, float b) { return (a > b) ? a : b; }
    inline float clamp(float a, float min_, float max_) {
        return min(max(a, min_), max_);
    }
	inline float sqrt(float a) { return std::sqrt(a); }
	inline float sin(float a) { return std::sin(a); }
	inline float cos(float a) { return std::cos(a); }
	inline float tan(float a) { return std::tan(a); }
	inline float asin(float a) { return std::asin(a); }
	inline float acos(float a) { return std::acos(a); }
	inline float atan(float a) { return std::atan(a); }
	inline float log(float a) { return std::log(a); }
	inline float exp(float a) { return std::exp(a); }
	inline float log2(float a) { return std::log2(a); }
	inline float exp2(float a) { return std::exp2(a); }
	inline float atan2(float a, float b) { return std::atan2(a, b); }
	inline float radians(float a) { return a * pi / 180; } // a is degrees, returns radians
	inline float degrees(float a) { return a * 180 / pi; } // a is radians, returns degrees
	inline float lerp(float a, float b, float u) { return a * (1 - u) + b * u; }
	inline bool quadratic(float a, float b, float c, float& r0, float& r1) {
		float q = b * b - 4 * a * c;
		if (q >= 0) {
			float sq = sqrt(q);
			float d = 1 / (2 * a);
			r0 = (-b + sq) * d;
			r1 = (-b - sq) * d;
			return true;
		}
		return false; // No roots
	}
	inline float clamp_rotation(float theta) {
		while (theta < 0.0f) {
			theta += two_pi;
		}
		return (fmod(theta, two_pi));
	}
	inline float angle_difference(float a1, float a2) {
		float v1 = abs(a1 - a2);
		float v2 = min(abs(two_pi - a1), a1) + min(abs(two_pi - a2), a2);
		return min(v1, v2);
	}
	inline float normalize_in_range(float val, float min_, float max_) {
		return (val - min_) / (max_ - min_);
	}
}