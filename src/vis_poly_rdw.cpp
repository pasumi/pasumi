#include <iostream>
#include <fstream>
#include <chrono>

#include "vis_poly_rdw.h"
#include "math.hpp"
#include "user.h"
#include "geometry.h"
#include "config.h"
#include "resetter.h"

vis_poly_rdw::vis_poly_rdw(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter) {
	name = "Vis. Poly. RDW";

	cur_rota_gain = 1.0f;
	min_rota_gain = 0.67f;
	max_rota_gain = 1.24f;
	prev_rota_gain = 1.0f;

	cur_trans_gain = 1.0f;
	min_trans_gain = 0.86f;
	max_trans_gain = 1.26f;

	curve_radius = 7.5f; // meters
	cur_curve_per_deg = curve_radius_to_deg_per_meter();
	curve_dir = 1; // curve to the left of the user == 1. curve to the right of the user == -1

	reset_policy = _resetter;

	prev_loss = -1.0f;
	cur_loss = -1.0f;
}

visibility_polygon vis_poly_rdw::get_vis_poly(vec2f pos, environment* env, float heading) {
	namespace V = Visibility;
	std::vector<V::Segment> segments;
	for (int i = 0; i < env->get_vertices().size(); i++) {
		vec2f* p1 = env->get_vertices()[i];
		vec2f* p2 = env->get_vertices()[(i + 1) % env->get_vertices().size()];
		V::Point pp1 = V::Point(p1->x, p1->y);
		V::Point pp2 = V::Point(p2->x, p2->y);
		segments.push_back(V::Segment(pp1, pp2));
	}
	for (obstacle* o : env->get_obstacles()) {
		for (int i = 0; i < o->get_vertices().size(); i++) {
			vec2f* p1 = o->get_vertices()[i];
			vec2f* p2 = o->get_vertices()[(i + 1) % o->get_vertices().size()];
			V::Point pp1 = V::Point(p1->x, p1->y);
			V::Point pp2 = V::Point(p2->x, p2->y);
			segments.push_back(V::Segment(pp1, pp2));
		}
	}

	segments = V::breakIntersections(segments);
	V::Point position(pos.x, pos.y);
	auto result = V::compute(position, segments);
	std::vector<vec2f*> verts;
	for (int i = 0; i < result.outer().size() - 1; i++) {
		verts.push_back(new vec2f(result.outer()[i].x(), result.outer()[i].y()));
	}

	return visibility_polygon(verts, new vec2f(pos.x, pos.y), heading);
}

void vis_poly_rdw::update_visibility_polygons(user* egocentric_user) {
	virt_vis_poly_center = egocentric_user->state.get_virt_pos();
	virt_vis_poly_theta = egocentric_user->state.get_virt_heading();
	//virt_vis_poly = get_vis_poly(virt_vis_poly_center, virt_arr_env);
	virt_vis_poly = get_vis_poly(virt_vis_poly_center, (environment*)(egocentric_user->virtual_env()), virt_vis_poly_theta);

	phys_vis_poly_center = egocentric_user->state.get_phys_pos();
	phys_vis_poly_theta = egocentric_user->state.get_phys_heading();
	//phys_vis_poly = get_vis_poly(phys_vis_poly_center, phys_arr_env);
	phys_vis_poly = get_vis_poly(phys_vis_poly_center, (environment*)(egocentric_user->physical_env()), phys_vis_poly_theta);

	std::chrono::milliseconds end = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()
		);
}

void vis_poly_rdw::orient_visibility_polygons(user* egocentric_user) {
	// Physical vis polygon
	// Rotate the polygon so that it matches with the user's heading direction (CGAL boolean
	// set operations do not consider orientation, so we must do that part ourselves).
	float theta = egocentric_user->state.get_phys_heading();
	theta = -(theta - (math::pi / 2.0f));
	for (vec2f* v : phys_vis_poly.verts) {
		float x = v->x;
		float y = v->y;
		float new_x = (x * math::cos(theta)) - (y * math::sin(theta));
		float new_y = (x * math::sin(theta)) + (y * math::cos(theta));
		v = new vec2f(new_x, new_y);
	}
	// Also rotate the observer's position inside the vis poly
	float x = phys_vis_poly.center->x;
	float y = phys_vis_poly.center->y;
	float new_x = (x * math::cos(theta)) - (y * math::sin(theta));
	float new_y = (x * math::sin(theta)) + (y * math::cos(theta));
	phys_vis_poly.center = new vec2f(new_x, new_y);
	// Translate the polygon so that the origin is the observer point of the vis poly
	for (vec2f* v : phys_vis_poly.verts) {
		v = new vec2f(v->x + -phys_vis_poly.center->x,
					  v->y + -phys_vis_poly.center->y);
	}

	// Virtual vis polygon
	// Rotate the polygon so that it matches with the user's heading direction (CGAL boolean
	// set operations do not consider orientation, so we must do that part ourselves).
	theta = egocentric_user->state.get_virt_heading();
	theta = -(theta - (math::pi / 2.0f));
	for (vec2f* v : virt_vis_poly.verts) {
		float x = v->x;
		float y = v->y;
		float new_x = (x * math::cos(theta)) - (y * math::sin(theta));
		float new_y = (x * math::sin(theta)) + (y * math::cos(theta));
		v = new vec2f(new_x, new_y);
	}
	// Also rotate the observer's position inside the vis poly
	x = virt_vis_poly.center->x;
	y = virt_vis_poly.center->y;
	new_x = (x * math::cos(theta)) - (y * math::sin(theta));
	new_y = (x * math::sin(theta)) + (y * math::cos(theta));
	virt_vis_poly.center = new vec2f(new_x, new_y);
	// Translate the polygon so that the origin is the observer point of the vis poly
	for (vec2f* v : virt_vis_poly.verts) {
		v = new vec2f(v->x + -virt_vis_poly.center->x,
					  v->y + -virt_vis_poly.center->y);
	}
}

redirection_unit vis_poly_rdw::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	//update_dynamic_obstacles(); // TODO: add support for dyunamic obstacles, and update the vertices of those obstalces in the environment objects
	update_visibility_polygons(egocentric_user);
	//orient_visibility_polygons(egocentric_user);
	//get_simple_visibility_polygons();
	set_steer_target(sim_state, egocentric_user);

	update_loss(sim_state, egocentric_user);

	redirection_unit next_redirection = set_gains(dx, dy, dtheta, sim_state, egocentric_user);

	prev_loss = cur_loss;
	return next_redirection;
}

void vis_poly_rdw::set_steer_target(simulation_state& sim_state, user* egocentric_user) {
	slice* virt_slice = virt_vis_poly.slices[0];
	float best_diff = math::max_float;

	for (slice* s : phys_vis_poly.slices) {
		if (phys_vis_poly.slices[0]->theta_offset <= math::pi * 0.5f) {
			if (math::abs(s->theta_offset) > math::pi * 0.5f) break;
		}
		float diff = math::abs(virt_slice->area - s->area);
		//float diff = get_symm_diff_area(s, virt_slice);
		//float diff = math::abs(virt_slice->width - s->width);
		if (diff < best_diff) {
			best_diff = diff;
			best_slice = s;
		}
	}
	assert(best_diff != math::max_float, "Could not find a best slice!");

	steer_target = egocentric_user->state.get_phys_pos() + best_slice->bisector;
}

void vis_poly_rdw::update_loss(simulation_state& sim_state, user* egocentric_user) {
	cur_north_loss = 0.0f;
	cur_east_loss = 0.0f;
	cur_west_loss = 0.0f;

	cur_loss = 0.0f;
}

float vis_poly_rdw::get_proximity_in_visibility_polygon(float dir, std::vector<vec2f> poly) {
	float best_dist = math::max_float;
	vec2f vec_dir = rad_2_vec(dir);
	
	for (int i = 0; i < poly.size(); i++) {
		vec2f p1 = poly[i];
		vec2f p2 = poly[(i+1)%poly.size()];
		float d = geom::ray_line_intersect(new vec2f(0.0f, 0.0f), &vec_dir, &p1, &p2); // Ray origin is (0, 0) because the visibility polygon has been recentered.
		if (d != -1.0f && d < best_dist) best_dist = d;
	}
	return best_dist;
}

redirection_unit vis_poly_rdw::set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	redirection_unit redir_unit;
	vec2f user_dir = rad_2_vec(egocentric_user->state.get_phys_heading());
	vec2f user_to_target = normalize(steer_target - egocentric_user->state.get_phys_pos());
	float angle_to_gradient = signed_angle(user_dir, user_to_target);

	if (dtheta) {
		// User has to turn right to face the gradient
		if (math::sign(angle_to_gradient) == -1) {
			// User is turning to the right
			if (dtheta < 0) {
				redir_unit.rota_gain = max_rota_gain;
				cur_rota_gain = max_rota_gain;
			}
			// User is turning to the left
			else {
				redir_unit.rota_gain = min_rota_gain;
				cur_rota_gain = min_rota_gain;
			}
		}
		// User has to turn left to face the gradient
		else {
			// User is turning to the right
			if (dtheta < 0) {
				redir_unit.rota_gain = min_rota_gain;
				cur_rota_gain = min_rota_gain;
			}
			// User is turning to the left
			else {
				redir_unit.rota_gain = max_rota_gain;
				cur_rota_gain = max_rota_gain;
			}
		}

		redir_unit.apply_rota = true;
		redir_unit.apply_trans = false;
		redir_unit.apply_curve = false;
		redir_unit.apply_bend = false;
	}
	else if (dx || dy) {
		// User has to turn right to face the gradient
		if (math::sign(angle_to_gradient) == -1) {
			redir_unit.curve_gain = cur_curve_per_deg;
			redir_unit.curve_gain_dir = -1;
			curve_dir = -1;
		}
		// User has to turn left to face the gradient
		else {
			redir_unit.curve_gain = cur_curve_per_deg;
			redir_unit.curve_gain_dir = 1;
			curve_dir = 1;
		}

		// Set the translation gain according to the amount of space in front of 
		// the user in the PE relative to the VE. This is basically the same as ARC.
		float virt_slice_height = virt_vis_poly.slices[0]->avg_height;
		float phys_slice_height = phys_vis_poly.slices[0]->avg_height;
		float trans_gain = phys_slice_height / virt_slice_height;
		trans_gain = math::clamp(trans_gain, min_trans_gain, max_trans_gain);
		redir_unit.trans_gain = trans_gain;
		cur_trans_gain = trans_gain;

		redir_unit.apply_rota = false;
		redir_unit.apply_trans = true;
		redir_unit.apply_curve = true;
		redir_unit.apply_bend = false;
	}

	apply_rota = redir_unit.apply_rota;
	cur_rota_gain = redir_unit.rota_gain; 
	apply_trans = redir_unit.apply_trans;
	cur_trans_gain = redir_unit.trans_gain;
	apply_curve = redir_unit.apply_curve;
	cur_curve_gain = redir_unit.curve_gain;
	curve_dir = redir_unit.curve_gain_dir;

	return redir_unit;
}

float vis_poly_rdw::set_rotation_gain(simulation_state& sim_state, user* egocentric_user) {
	return 0.0f;
}

float vis_poly_rdw::set_translation_gain(simulation_state& sim_state, user* egocentric_user) {
	float trans_gain = phys_north_loss / virt_north_loss;
	trans_gain = math::clamp(trans_gain, min_trans_gain, max_trans_gain);

	return trans_gain;
}

float vis_poly_rdw::set_curvature_gain(int direction) {
	float scale;
	if (direction == CURVE_TO_LEFT) {
		scale = math::min(1.0f, cur_west_loss);
	}
	else {
		scale = math::min(1.0f, cur_east_loss);
	}
	return math::min(cur_curve_per_deg * math::abs(scale), cur_curve_per_deg);
}

float vis_poly_rdw::distance_to_closest_feature(vec2f pos, vec2f dir, visibility_polygon vis_poly) {
	float min_dist = math::max_float;
	for (int i = 0; i < vis_poly.verts.size(); i++) {
		vec2f* p1 = vis_poly.verts[i % vis_poly.verts.size()];
		vec2f* p2 = vis_poly.verts[(i + 1) % vis_poly.verts.size()];
		float t = geom::ray_line_intersect(&(vec2f(0.0f, 0.0f)), &dir, p1, p2);
		if (t >= 0.0f && t < min_dist) {
			min_dist = t;
		}
	}

	return min_dist;
}

std::vector<std::vector<vec2f>> vis_poly_rdw::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
	std::vector<std::vector<vec2f>> test;
	return test;
}