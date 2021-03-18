#pragma once

#include <vector>

#include "vec2f.hpp"
#include "geometry.h"

class polygon {
public:
	struct segment {
		vec2f p1;
		vec2f p2;
	};

	polygon();
	polygon(std::vector<segment> boundary, std::vector<segment> holes);

protected:
	std::vector<segment> boundary;
	std::vector<segment> holes;
	std::vector<vec2f*> boundary_pts;

private:

};