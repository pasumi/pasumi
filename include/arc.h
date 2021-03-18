#pragma once

//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Boolean_set_operations_2.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Triangular_expansion_visibility_2.h>
//#include <CGAL/Rotational_sweep_visibility_2.h>
//#include <CGAL/Arr_segment_traits_2.h>
//#include <CGAL/Arrangement_2.h>

#include "redirector.h"
#include "graph.h"
#include "visibility_polygon.h"
#include "VisibilityPolygon.h"
#include "trans_view.h"

class traj_node {
public:
	traj_node() {}

	traj_node(trajectory_unit orig_path_unit) {
		original_pos = vec2f(orig_path_unit.x, orig_path_unit.y);
		original_theta = orig_path_unit.theta;
	}

	~traj_node() {

	}

	void set_child(traj_node* c) {
		child = c;
	}

	vec2f original_pos; // original position in path (global position)
	float original_theta; // original global theta 
	vec2f dir_from_parent;
	vec2f dir_to_child;
	float pos_offset; // distance from parent node
	float theta_offset; // angle offset from parent node
	vec2f trans_pos; // position in path after transforming (Redirection)
	float trans_theta;
	traj_node* child;
	float curvature_gain_applied;
	int curvature_gain_dir;
	float trans_gain_applied;

private:
};

class arc : public redirector {
public:
	arc(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter);
	~arc();
	void build_visibility_graphs(physical_environment* phys_env, virtual_environment* virt_env);
	redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	std::vector<std::vector<vec2f>> get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue);

	visibility_polygon* phys_vis_poly;
	std::vector<vec2f> simple_phys_vis_poly;
	vec2f phys_vis_poly_center; // The point of the observer in the vis poly
	float phys_vis_poly_theta; // The user's orientation inside the vis poly
	visibility_polygon* virt_vis_poly;
	std::vector<vec2f> simple_virt_vis_poly;
	vec2f virt_vis_poly_center; // The point of the observer in the vis poly
	float virt_vis_poly_theta; // The user's orientation inside the vis poly
	std::vector<float> sample_directions;
	std::vector<float> reset_sample_directions;

	float constructor_time = 0.0f;
	float get_vis_poly_time = 0.0f;
	float update_visibility_polygons_time = 0.0f;
	float update_time = 0.0f;
	float get_loss_gradient_direction_time = 0.0f;
	float get_simple_visibility_polygons_time = 0.0f;
	float update_loss_time = 0.0f;
	float get_ray_weight_time = 0.0f;
	float get_proximity_in_visibility_polygon_time = 0.0f;
	float area_of_polygon_with_holes_time = 0.0f;
	float compute_rotation_loss_time = 0.0f;
	float set_gains_time = 0.0f;
	float reset_time = 0.0f;
	float reset_to_distance_alignment_time = 0.0f;
	float get_current_visibility_polygon_time = 0.0f;
	float distance_to_closest_feature_time = 0.0f;
	float reset_to_gradient_time = 0.0f;
	float compute_gradient_time = 0.0f;
	float compute_repulsive_force_time = 0.0f;
	slice test;
	trans_view phys_trans_view;
	trans_view virt_trans_view;

private:
	redirection_unit set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	float set_rotation_gain(simulation_state& sim_state, user* egocentric_user);
	float set_translation_gain(simulation_state& sim_state, user* egocentric_user);
	float set_curvature_gain(int direction);
	float distance_to_closest_feature(vec2f pos, vec2f dir, visibility_polygon vis_poly);
	void update_loss(simulation_state& sim_state, user* egocentric_user);
	void update_visibility_polygons(user* egocentric_user);
	//visibility_polygon get_vis_poly(vec2f pos, Arrangement_2 arr_env);
	visibility_polygon* get_vis_poly(vec2f pos, environment* env, float heading);
	void orient_visibility_polygons(user* egocentric_user);
	float get_proximity_in_visibility_polygon(float dir, std::vector<vec2f> poly);
	void set_steer_target(simulation_state& sim_state, user* egocentric_user);
	float get_symm_diff_area(slice* phys_slice, slice* virt_slice);
	void compute_proximities(simulation_state& sim_state, user* egocentric_user);
	redirection_unit get_proximity_steering_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	redirection_unit get_visibility_slice_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	redirection_unit gain_calculation(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	void update_prediction(user* egocentric_user);
	void handle_path_collision(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	bool check_for_collision(simulation_state& sim_state, user* egocentric_user, int& collision_index);
	void resolve_path_on_collision(simulation_state& sim_state, user* egocentric_user, int collision_index);
	float compute_minimum_curvature(simulation_state& sim_state, user* egocentric_user, int collision_index);
	bool transform_path(simulation_state& sim_state, user* egocentric_user, int collision_index, float deg_per_meter, std::vector<traj_node*>& transformed_path);

	//============================
	// Gradient stuff
	void reset_to_gradient(vec2f phys_heading, float virt_heading, std::vector<trajectory_unit>* path, std::deque<proximity_container*> prox_queue, physical_environment* phys_env, vec2f phys_pos, vec2f virt_pos);
	vec2f compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos);
	float compute_repulsive_force(vec2f pos, std::vector<object*> objects);
	std::vector<float> theta_values;
	float SFR2G_STEP_SIZE = 0.1f;
	const int GRADIENT_SAMPLE_RATE = 8; // Number of points around the user to check the value of the potential field
	vec2f gradient_dir;
	//============================

	const float PREDICTION_TIME_HORIZON = 1.0f;
	const float REDIRECTION_DISTANCE_THRESHOLD = 0.25f;
	const int NUM_RESET_SAMPLE_RAYS = 20;
	const float FORWARD_DISTANCE_WEIGHT = 5.0f;
	const float REDIRECTION_LOSS_THRESHOLD = 0.1f;
	const float ROTA_GAIN_SMOOTHING = 0.125f;
	const float VISUAL_FIELD = math::radians(210.0f); // radians
	const float FORWARD_ANGLE = math::pi / 2;
	const float LEFT_ANGLE = math::pi;
	const float RIGHT_ANGLE = 0.0f;
	const int NUM_SAMPLE_RAYS = 10; // Excluding the extents of the visual field.
	const bool LOWER_FREQUENCY = false;
	const float CALCULATION_FREQUENCY = 10.0f; // In seconds
	const float STEERING_ANGLE_THRESHOLD = 0.0872665f; // 5 degrees. Determines when to stop steering by visibility polygon, and instead steer by ARC.

	vec2f loss_gradient_dir;
	float prev_loss;
	float prev_north_loss;
	float prev_east_loss;
	float prev_west_loss;
	float phys_north_loss;
	float phys_east_loss;
	float phys_west_loss;
	float virt_north_loss;
	float virt_east_loss;
	float virt_west_loss;
	float cur_loss;
	float cur_north_loss;
	float cur_east_loss;
	float cur_west_loss;
	float prev_rota_gain;
	float calculation_freq_counter;
	redirection_unit prev_redirection;
	vec2f steer_target;
	slice* best_slice;
	bool steer_by_slice;
	bool aligned_with_slice_recently; // Flag to track if we got the user to look at the steering target since the last visibility event happened
	float phys_dist_north;
	float virt_dist_north;

	float ORACLE_TIME_FRAME = 3.0f; // in seconds
	std::vector<traj_node*> predicted_path;
	std::vector<traj_node*> transformed_path;
};