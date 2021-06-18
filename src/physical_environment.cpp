#include <iostream>
#include <fstream>
#include <sstream>

#include "physical_environment.h"
#include "geometry.h"
#include "config.h"
#include "dynamic_obstacle.h"

physical_environment::physical_environment() {

}

physical_environment::physical_environment(fs::path env_file) {
	this->load_xml_file(env_file);
}

void physical_environment::step() {
	for (obstacle* o : obstacles) {
		o->step();
	}
}

vec2f physical_environment::get_center() {
	return center;
}