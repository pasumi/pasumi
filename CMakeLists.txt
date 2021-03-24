# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ("pasumi")

include_directories(
  # Pasumi include files
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# The following lines build the main executable. If you add a source
# code file to Pasumi, be sure to include it in this list.
add_executable(pasumi
  # Header files
  include/arc.h
  include/config.h
  include/dynamic_obstacle.h
  include/environment.h
  include/geometry.h
  include/history.h
  include/math.hpp
  
  include/motion_model.h
  
  include/object.h
  include/obstacle.h
  include/physical_environment.hpp
  include/polygon.h
  include/primitives.hpp
  include/proximity_container.hpp
  include/redirector.h
  include/resetter.h
  include/reset_to_forward_distance.h
  
  
  include/simulation.hpp
  include/simulation_state.h
  
  include/timestep.h
  include/user.hpp
  include/user_state.hpp
  include/vec2f.hpp
  include/vector2.hpp
  include/virtual_environment.hpp
  include/wall.h

  # Source code files
  src/arc.cpp
  src/config.cpp
  src/dynamic_obstacle.cpp
  src/environment.cpp
  src/history.cpp
  
  src/motion_model.cpp
  
  src/obstacle.cpp
  src/physical_environment.cpp
  src/polygon.cpp
  src/proximity_container.cpp
  src/redirector.cpp
  src/resetter.cpp
  src/reset_to_forward_distance.cpp
  
  
  src/simulation.cpp
  src/simulation_state.cpp
  
  src/timestep.cpp
  src/user.cpp
  src/user_state.cpp
  src/virtual_environment.cpp
  src/wall.cpp
 "src/main.cpp")


# Include sub-projects.
# add_subdirectory ("pasumi")
