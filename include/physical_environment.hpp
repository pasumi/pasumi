#pragma once

#include <vector>

#include "vec2f.hpp"
#include "wall.h"
#include "obstacle.h"
#include "environment.h"

class physical_environment : public environment {
    public:
        physical_environment();
        physical_environment(std::vector<vec2f*> verts, std::vector<obstacle*> obstacles, vec2f center, char* name);
        physical_environment(std::string boundary_file, std::string obstacles_file, char* name);
        /*
        std::vector<vec2f> get_vertices();
        std::vector<wall*> get_walls();
        std::vector<obstacle*> get_obstacles();
        bool point_is_legal(vec2f p);
        bool line_is_legal(vec2f* p1, vec2f* p2);
        */
        vec2f get_center();
        int vertex_buffer_size();
        int index_buffer_size();
        int wall_vertex_buffer_size();
        int wall_index_buffer_size();
        int obstacle_vertex_buffer_size();
        int obstacle_index_buffer_size();
        vec2f sample_point();
        void step();
        
        int num_vertices;
        std::vector<float> gl_wall_verts;
        std::vector<float> gl_obstacle_verts;
        std::vector<unsigned int> gl_wall_indices;
        std::vector<unsigned int> gl_obstacle_indices;
        float min_x;
        float max_x;
        float min_y;
        float max_y;

    private:
        vec2f center;
        /*
        std::vector<vec2f> verts; // Vertices of boundary in counter clockwise order
        std::vector<wall*> walls;
        std::vector<obstacle*> obstacles;
        */
};