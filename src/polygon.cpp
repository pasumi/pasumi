#include "polygon.h"

polygon::polygon() {

}

polygon::polygon(std::vector<segment> boundary, std::vector<segment> holes) {
	this->boundary = boundary;
	this->holes = holes;
	for (segment s : boundary) {
		boundary_pts.push_back(&(s.p1));
	}
}