#pragma once

#include "redirector.h"
#include "graph.h"
#include "visibility_polygon.h"

class arc_simple : public redirector {
public:
	arc_simple(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter);
	redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	std::vector<std::vector<vec2f>> get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue);

	visibility_polygon cur_phys_vis_poly;
	visibility_polygon cur_virt_vis_poly;
	std::vector<float> sample_directions;

private:
	redirection_unit set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	float set_translation_gain(simulation_state& sim_state, user* egocentric_user);
	float set_curvature_gain(int direction);

	float distance_to_closest_feature(vec2f pos, vec2f dir, visibility_polygon vis_poly);
	vec2f get_optimal_heading(vec2f pos, visibility_polygon vis_poly, float dist_north, float dist_east, float dist_south, float dist_west);
	float attenuate_gain(float angle_to_target, float cur_gain);

	void update_losses(simulation_state& sim_state, user* egocentric_user);
	float get_distance_in_direction(vec2f pos, vec2f dir, environment* env);

	//============================
	// Gradient stuff
	void reset_to_gradient(vec2f phys_heading, float virt_heading, std::vector<trajectory_unit>* path, std::deque<proximity_container*> prox_queue, physical_environment* phys_env, vec2f phys_pos, vec2f virt_pos);
	vec2f compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos);
	float compute_repulsive_force(vec2f pos, std::vector<object*> objects);
	std::vector<float> theta_values;
	float SFR2G_STEP_SIZE = 0.1f;
	const int GRADIENT_SAMPLE_RATE = 120; // Number of points around the user to check the value of the potential field
	vec2f gradient_dir;
	//============================

	const float VIS_NODE_RESOLUTION = 1.0f;
	const float PREDICTION_TIME_HORIZON = 1.0f;
	const float REDIRECTION_DISTANCE_THRESHOLD = 0.25f;
	const float MAX_NODE_DISTANCE = 4.0f;
	const int SAMPLE_RATE = 20;
	const float FORWARD_DISTANCE_WEIGHT = 5.0f;
	const float REDIRECTION_LOSS_THRESHOLD = 0.1f;
	const float ROTA_GAIN_SMOOTHING = 0.125f;
	const bool LOWER_FREQUENCY = false;
	const float CALCULATION_FREQUENCY = 10.0f; // In seconds

	float prev_loss;
	float prev_north_loss;
	float prev_east_loss;
	float prev_west_loss;
	float cur_loss;
	float cur_north_loss;
	float cur_east_loss;
	float cur_west_loss;
	float prev_rota_gain;
	float calculation_freq_counter; 
	redirection_unit prev_redirection;
};