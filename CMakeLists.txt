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
  include/apf_grad.h
  include/apf_vec.h
  include/arc.h
  include/arc.h
  include/config.h
  include/dynamic_obstacle.h
  include/edge.h
  include/edge_pq.h
  include/environment.h
  include/floats.hpp
  include/geometry.h
  include/graph.h
  include/history.h
  include/linked_list.h
  include/ll_node.h
  include/math.hpp
  include/modified_reset_to_center.h
  include/motion_model.h
  include/node.h
  include/no_rdw.h
  include/object.h
  include/obstacle.h
  include/physical_environment.hpp
  include/polygon.h
  include/polygon_match.h
  include/primitives.hpp
  include/print_utils.h
  include/proximity_container.hpp
  include/redirector.h
  include/resetter.h
  include/reset_to_forward_distance.h
  include/reset_to_gradient.h
  include/s2c.h
  include/shader.h
  include/shader_m.h
  include/shader_s.h
  include/simulation.hpp
  include/simulation_state.h
  include/step_forward_reset_to_gradient.h
  include/timestep.h
  include/trans_view.h
  include/user.hpp
  include/user_state.hpp
  include/vec2f.hpp
  include/vector2.hpp
  include/virtual_environment.hpp
  include/visibility.hpp
  include/visibility_polygon.h
  include/wall.h

  # Source code files
  src/apf_grad.cpp
  src/apf_vec.cpp
  src/arc.cpp
  src/arc.cpp
  src/config.cpp
  src/dynamic_obstacle.cpp
  src/edge.cpp
  src/edge_pq.cpp
  src/environment.cpp
  src/graph.cpp
  src/history.cpp
  src/linked_list.cpp
  src/ll_node.cpp
  src/main.cpp
  src/modified_reset_to_center.cpp
  src/motion_model.cpp
  src/node.cpp
  src/no_rdw.cpp
  src/obstacle.cpp
  src/physical_environment.cpp
  src/polygon.cpp
  src/proximity_container.cpp
  src/redirector.cpp
  src/resetter.cpp
  src/reset_to_forward_distance.cpp
  src/reset_to_gradient.cpp
  src/s2c.cpp
  src/simulation.cpp
  src/simulation_state.cpp
  src/step_forward_reset_to_gradient.cpp
  src/timestep.cpp
  src/trans_view.cpp
  src/user.cpp
  src/user_state.cpp
  src/virtual_environment.cpp
  src/visibility_polygon.cpp
  src/wall.cpp
  
)


# Include sub-projects.
# add_subdirectory ("pasumi")
