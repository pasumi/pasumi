#pragma once

#include <vector>

class visibility_polygon;

#include "vec2f.hpp"
#include "obstacle.h"
#include "geometry.h"
#include "wall.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
// Triangulation imports
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

// ** Visibility polygon stuff **
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
// For symmetric difference
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;
// For visibility polygon
typedef Kernel::Segment_2                                       Segment_2;
typedef CGAL::Arr_segment_traits_2<Kernel>                      Traits_2;
typedef CGAL::Arrangement_2<Traits_2>                           Arrangement_2;
typedef Arrangement_2::Halfedge_const_handle                    Halfedge_const_handle;
typedef Arrangement_2::Face_handle                              Face_handle;
// Define the used visibility class
typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>  TEV;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;

class environment {
    public:
        typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
        typedef CGAL::Triangulation_vertex_base_2<K> Vb;
        typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
        typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
        typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;

        environment();
        ~environment();

        std::vector<vec2f*> get_vertices();
        std::vector<Segment_2> get_cgal_verts();
        std::vector<wall*> get_walls();
        std::vector<obstacle*> get_obstacles();
        bool point_is_legal(vec2f p);
        bool line_is_legal(vec2f* p1, vec2f* p2);
        float get_closest_obstacle_distance(vec2f pt);
        float get_distance_in_direction_from_point(vec2f p, vec2f dir);
        virtual void step() = 0;
        virtual vec2f sample_point() = 0;

        CDT triangulate();
        void compute_features(char* name);
        void compute_area();
        Polygon_2 compute_vis_poly(vec2f pos);
        void export_visibility_polygon_data(std::vector<visibility_polygon*> vis_polys, char* name);

        char* name;
        float min_x;
        float max_x;
        float min_y;
        float max_y;
        bool unbounded;
        float area;
        Polygon_2 cgal_outP;
        std::vector<Polygon_2> cgal_holesP;
        Polygon_with_holes_2 cgal_pwh;

    protected:
        void build_cgal_data();

        std::vector<vec2f*> verts; // Vertices of boundary in counter clockwise order
        std::vector<wall*> walls;
        std::vector<obstacle*> obstacles;
        std::vector<Segment_2> cgal_verts; // includes obstacle vertices
        int NUM_POINTS_TO_SAMPLE = 1000; // For environment statistics
};