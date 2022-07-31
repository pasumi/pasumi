#pragma once

#include "redirector.h"
#include "resetter.h"
#include "visibility_polygon.h"
#include "VisibilityPolygon.h"

class vis_poly_rdw : public redirector {
public:
	vis_poly_rdw(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter);
	redirection_unit update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	std::vector<std::vector<vec2f>> get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue);

	visibility_polygon phys_vis_poly;
	vec2f phys_vis_poly_center; // The point of the observer in the vis poly
	float phys_vis_poly_theta; // The user's orientation inside the vis poly
	visibility_polygon virt_vis_poly;
	vec2f virt_vis_poly_center; // The point of the observer in the vis poly
	float virt_vis_poly_theta; // The user's orientation inside the vis poly

private:
	redirection_unit set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user);
	float set_rotation_gain(simulation_state& sim_state, user* egocentric_user);
	float set_translation_gain(simulation_state& sim_state, user* egocentric_user);
	float set_curvature_gain(int direction);
	float distance_to_closest_feature(vec2f pos, vec2f dir, visibility_polygon vis_poly);
	void update_loss(simulation_state& sim_state, user* egocentric_user);
	void update_visibility_polygons(user* egocentric_user);
	visibility_polygon get_vis_poly(vec2f pos, environment* env, float heading);
	void orient_visibility_polygons(user* egocentric_user);
	float get_proximity_in_visibility_polygon(float dir, std::vector<vec2f> poly);
	void set_steer_target(simulation_state& sim_state, user* egocentric_user);

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

	vec2f steer_target;
	slice* best_slice;
};