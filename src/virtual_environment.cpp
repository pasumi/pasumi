#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>

#include "virtual_environment.h"
#include "geometry.h"
#include "config.h"

virtual_environment::virtual_environment() {

}

virtual_environment::virtual_environment(fs::path env_file) {
	this->load_xml_file(env_file);
}

void virtual_environment::step() {
	for (obstacle* o : obstacles) {
		o->step();
	}
}