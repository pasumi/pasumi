#include <iostream>
#include <fstream>
#include <chrono>

#include <CGAL/number_utils.h>
#include <CGAL/Cartesian.h>
#include "print_utils.h"
typedef CGAL::Cartesian<double> K;
typedef CGAL::Aff_transformation_2<K> Transformation;
typedef CGAL::Point_2<K> Point;
typedef CGAL::Vector_2<K> Vector;
typedef CGAL::Direction_2<K> Direction;

#include "arc.h"
#include "math.hpp"
#include "user.hpp"
#include "geometry.h"
#include "config.h"

//#include "visibility.hpp"
#include "vector2.hpp"

arc::arc(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter) {
	time_t start_time;
	start_time = time(NULL);

	name = "ISMAR RDW";

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

	// Get the angles for distance sampling within the user's field of vision
	float interval_size = VISUAL_FIELD / NUM_SAMPLE_RAYS;
	float starting_angle = (math::pi / 2.0f) - (VISUAL_FIELD / 2.0f);
	sample_directions.push_back(starting_angle);
	for (int i = 1; i < NUM_SAMPLE_RAYS; i++) {
		sample_directions.push_back(starting_angle + (interval_size * i));
	}
	sample_directions.push_back(VISUAL_FIELD);
	sample_directions.push_back(FORWARD_ANGLE);
	sample_directions.push_back(LEFT_ANGLE);
	sample_directions.push_back(RIGHT_ANGLE);

	reset_policy = _resetter;

	// Get the angles for distance sampling for resetting
	for (int i = 0; i < NUM_RESET_SAMPLE_RAYS; i++) {
		reset_sample_directions.push_back(((2 * math::pi) / NUM_RESET_SAMPLE_RAYS) * i);
	}

	prev_loss = -1.0f;
	cur_loss = -1.0f;

	cur_alignment = std::vector<float>{ 0.0f, 0.0f, 0.0f };
	// TODO: need to figure out what to do about this. the cur_alignment variable is a 3-tuple since it's the sample based method that i used initially. but with the vis poly, the alignment is just 1 number (sum of areas of symm diff). idk the best way to fix the code to deal with this. I might just be able to ignore cur_alignment in this class and only care about cur_loss and prev_loss.

	best_slice = nullptr;
	aligned_with_slice_recently = false;

	//============================
	// Gradient stuff
	for (int i = 0; i < GRADIENT_SAMPLE_RATE; i++) {
		theta_values.push_back(((2 * math::pi) / GRADIENT_SAMPLE_RATE) * i);
	}

	calculation_freq_counter = CALCULATION_FREQUENCY;

	time_t end_time;
	end_time = time(NULL);
	constructor_time += end_time - start_time;
}

arc::~arc() {
	delete phys_vis_poly;
	delete virt_vis_poly;
}

//visibility_polygon arc::get_vis_poly(vec2f pos, Arrangement_2 arr_env) {
visibility_polygon* arc::get_vis_poly(vec2f pos, environment* env, float heading) {
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

	visibility_polygon* ret_val = new visibility_polygon(verts, &vec2f(pos.x, pos.y), heading);
	for (auto v : verts) {
		delete v;
	}
	return ret_val;
}

void arc::update_visibility_polygons(user* egocentric_user) {
	virt_vis_poly_center = egocentric_user->state.get_virt_pos();
	virt_vis_poly_theta = egocentric_user->state.get_virt_heading();
	//virt_vis_poly = get_vis_poly(virt_vis_poly_center, virt_arr_env);
	delete virt_vis_poly;
	virt_vis_poly = get_vis_poly(virt_vis_poly_center, (environment*)(egocentric_user->virtual_env()), virt_vis_poly_theta);

	phys_vis_poly_center = egocentric_user->state.get_phys_pos();
	phys_vis_poly_theta = egocentric_user->state.get_phys_heading();
	//phys_vis_poly = get_vis_poly(phys_vis_poly_center, phys_arr_env);
	delete phys_vis_poly;
	phys_vis_poly = get_vis_poly(phys_vis_poly_center, (environment*)(egocentric_user->physical_env()), phys_vis_poly_theta);
}

void arc::orient_visibility_polygons(user* egocentric_user) {
	// Physical vis polygon
	// Rotate the polygon so that it matches with the user's heading direction (CGAL boolean
	// set operations do not consider orientation, so we must do that part ourselves).
	float theta = egocentric_user->state.get_phys_heading();
	theta = -(theta - (math::pi / 2.0f));
	for (vec2f* v : phys_vis_poly->verts) {
		float x = v->x;
		float y = v->y;
		float new_x = (x * math::cos(theta)) - (y * math::sin(theta));
		float new_y = (x * math::sin(theta)) + (y * math::cos(theta));
		v = new vec2f(new_x, new_y);
	}
	// Also rotate the observer's position inside the vis poly
	float x = phys_vis_poly->center->x;
	float y = phys_vis_poly->center->y;
	float new_x = (x * math::cos(theta)) - (y * math::sin(theta));
	float new_y = (x * math::sin(theta)) + (y * math::cos(theta));
	phys_vis_poly->center = new vec2f(new_x, new_y);
	// Translate the polygon so that the origin is the observer point of the vis poly
	for (vec2f* v : phys_vis_poly->verts) {
		v = new vec2f(v->x + -phys_vis_poly->center->x,
					  v->y + -phys_vis_poly->center->y);
	}

	// Virtual vis polygon
	// Rotate the polygon so that it matches with the user's heading direction (CGAL boolean
	// set operations do not consider orientation, so we must do that part ourselves).
	theta = egocentric_user->state.get_virt_heading();
	theta = -(theta - (math::pi / 2.0f));
	for (vec2f* v : virt_vis_poly->verts) {
		float x = v->x;
		float y = v->y;
		float new_x = (x * math::cos(theta)) - (y * math::sin(theta));
		float new_y = (x * math::sin(theta)) + (y * math::cos(theta));
		v = new vec2f(new_x, new_y);
	}
	// Also rotate the observer's position inside the vis poly
	x = virt_vis_poly->center->x;
	y = virt_vis_poly->center->y;
	new_x = (x * math::cos(theta)) - (y * math::sin(theta));
	new_y = (x * math::sin(theta)) + (y * math::cos(theta));
	virt_vis_poly->center = new vec2f(new_x, new_y);
	// Translate the polygon so that the origin is the observer point of the vis poly
	for (vec2f* v : virt_vis_poly->verts) {
		v = new vec2f(v->x + -virt_vis_poly->center->x,
					  v->y + -virt_vis_poly->center->y);
	}
}

void arc::update_prediction(user* egocentric_user) {
	vec2f cur_phys_pos = egocentric_user->state.get_phys_pos();
	float cur_phys_heading = egocentric_user->state.get_phys_heading();
	vec2f cur_virt_pos = egocentric_user->state.get_virt_pos();
	float cur_virt_heading = egocentric_user->state.get_virt_heading();

	int num_timesteps_to_predict = ORACLE_TIME_FRAME / timestep::dt;
	int num_steps_remaining = egocentric_user->state.path.size();
	num_timesteps_to_predict = math::min(num_timesteps_to_predict, num_steps_remaining);
	predicted_path.clear();
	for (int i = 0; i < num_timesteps_to_predict; i++) {
		trajectory_unit pasted_path = egocentric_user->state.path[i];
		// Center it
		pasted_path.x -= cur_virt_pos.x;
		pasted_path.y -= cur_virt_pos.y;
		// Rotate it
		float theta_diff = cur_virt_heading - cur_phys_heading;
		pasted_path.theta += theta_diff;
		// Offset to the phys pos
		pasted_path.x += cur_phys_pos.x;
		pasted_path.y += cur_phys_pos.y;

		predicted_path.push_back(new traj_node(pasted_path));
	}
}

void arc::handle_path_collision(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	int collision_index = 0;
	bool has_collision = check_for_collision(sim_state, egocentric_user, collision_index);
	// No collision: don't change the path, and just follow it.
	if (!has_collision) {
		return;
	}
	// Apply redirection to transform the path in order to avoid the collision.
	else {
		resolve_path_on_collision(sim_state, egocentric_user, collision_index);
	}
	int t =5;
}

void arc::resolve_path_on_collision(simulation_state& sim_state, user* egocentric_user, int collision_index) {
	auto point_of_collision = predicted_path[collision_index];
	float minimum_curvature = compute_minimum_curvature(sim_state, egocentric_user, collision_index);
}

float arc::compute_minimum_curvature(simulation_state& sim_state, user* egocentric_user, int collision_index) {
	transformed_path.clear();
	bool transformed_successfully = transform_path(sim_state, egocentric_user, collision_index, cur_curve_per_deg, transformed_path);
	return 1.0f;
}

bool arc::transform_path(simulation_state& sim_state, user* egocentric_user, int collision_index, float deg_per_meter, std::vector<traj_node*>& transformed_path) {
	if (collision_index == 0) {
		traj_node* first = predicted_path[0];
		transformed_path.push_back(first);
		float t = egocentric_user->get_phys_env()->get_closest_obstacle_distance(first->trans_pos);
		return egocentric_user->get_phys_env()->get_closest_obstacle_distance(first->trans_pos) > config::RESET_DISTANCE_CHECK_VALUE;
	}
	float trans_to_apply = min_trans_gain;
	float curvature_to_apply = deg_per_meter;
	//float curvature_to_apply = 0.0f;
	int curvature_dir = CURVE_TO_LEFT;
	for (int i = 0; i < collision_index; i++) {
		if (i == 0) {
			auto cur = predicted_path[i];
			float offset_length = length(cur->original_pos - egocentric_user->state.get_phys_pos());
			offset_length *= trans_to_apply;
			float new_theta = cur->original_theta + (math::radians(curvature_to_apply)* i * offset_length * curvature_dir);
			vec2f origin = egocentric_user->state.get_phys_pos();
			vec2f new_pos = origin + vec2f(offset_length * math::cos(new_theta), offset_length * math::sin(new_theta));
			traj_node* first = new traj_node(trajectory_unit(cur->original_pos.x, cur->original_pos.y, cur->original_theta));
			first->trans_pos = new_pos;
			first->curvature_gain_applied = curvature_to_apply;
			first->curvature_gain_dir = curvature_dir;
			first->trans_gain_applied = trans_to_apply;
			transformed_path.push_back(first);
		}
		else {
			auto cur = predicted_path[i];
			auto prev = transformed_path.back();
			float offset_length = length(predicted_path[i]->original_pos - predicted_path[i - 1]->original_pos);
			offset_length *= trans_to_apply;
			float new_theta = cur->original_theta + (math::radians(curvature_to_apply)* i * offset_length * curvature_dir);
			vec2f origin = prev->trans_pos;
			vec2f new_pos = origin + vec2f(offset_length * math::cos(new_theta), offset_length * math::sin(new_theta));

			traj_node* new_node = new traj_node(trajectory_unit(cur->original_pos.x, cur->original_pos.y, cur->original_theta));
			new_node->trans_pos = new_pos;
			new_node->curvature_gain_applied = curvature_to_apply;
			new_node->curvature_gain_dir = curvature_dir;
			new_node->trans_gain_applied = trans_to_apply;
			transformed_path.push_back(new_node);
		}
		if (i > 0) {
			auto prev = transformed_path[transformed_path.size() - 2];
			auto cur = transformed_path[transformed_path.size() - 1];
			auto prev_to_cur = cur->trans_pos - prev->trans_pos;
			transformed_path[transformed_path.size() - 2]->trans_theta = vec_2_rad(normalize(prev_to_cur));
			int t = 4;
		}
	}
	vec2f final_pos = transformed_path.back()->trans_pos;
	return egocentric_user->get_phys_env()->get_closest_obstacle_distance(final_pos) > config::RESET_DISTANCE_CHECK_VALUE;
}

bool arc::check_for_collision(simulation_state& sim_state, user* egocentric_user, int& collision_index) {
	for (int i = 0; i < predicted_path.size(); i++) {
		vec2f pos = predicted_path[i]->original_pos;
		bool too_close = egocentric_user->get_phys_env()->get_closest_obstacle_distance(pos) < config::RESET_DISTANCE_CHECK_VALUE;
		if (too_close) {
			collision_index = i;
			return true;
		}
	}
	collision_index = -1;
	return false;
}

redirection_unit arc::update(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	update_prediction(egocentric_user);
	handle_path_collision(dx, dy, dtheta, sim_state, egocentric_user);
	auto first_rdw_step = transformed_path[0];
	redirection_unit next_redirection;
	next_redirection.apply_rota = false;
	next_redirection.apply_curve = true;
	next_redirection.curve_gain = first_rdw_step->curvature_gain_applied;
	next_redirection.curve_gain_dir = first_rdw_step->curvature_gain_dir;
	next_redirection.apply_trans = true;
	next_redirection.trans_gain = first_rdw_step->trans_gain_applied;
	next_redirection.apply_bend = false;
	return next_redirection;

	if (timestep::num_timesteps == 0) {
		phys_trans_view = trans_view(egocentric_user->get_phys_env(), 
									 egocentric_user->get_phys_pos());
		virt_trans_view = trans_view(sim_state.virt_env, egocentric_user->get_virt_pos());
	}

	// Don't compute gains on every frame
	if (LOWER_FREQUENCY) {
		calculation_freq_counter -= timestep::dt;

		if (calculation_freq_counter <= 0.0f) {
			redirection_unit next_redirection = gain_calculation(dx, dy, dtheta, sim_state, egocentric_user);

			prev_loss = cur_loss;
			prev_redirection = next_redirection;

			return next_redirection;
		}
		else {
			return prev_redirection;
		}
	}
	// Compute gains on each frame, as normal
	else { 
		return gain_calculation(dx, dy, dtheta, sim_state, egocentric_user);
	}
}

redirection_unit arc::gain_calculation(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	if (timestep::num_timesteps == 31) {
		int t = 4;
	}

	// At the start, compute the best slice and start steering towards it.
	if (timestep::num_timesteps == 0) {
		vec2f user_dir = rad_2_vec(egocentric_user->state.get_phys_heading());
		vec2f user_to_target = normalize(steer_target - egocentric_user->state.get_phys_pos());
		float angle_to_target = signed_angle(user_dir, user_to_target);
		update_visibility_polygons(egocentric_user);
		set_steer_target(sim_state, egocentric_user);

		steer_by_slice = math::abs(angle_to_target) >= STEERING_ANGLE_THRESHOLD;
		aligned_with_slice_recently = steer_by_slice;

		compute_proximities(sim_state, egocentric_user);
		redirection_unit next_redirection = set_gains(dx, dy, dtheta, sim_state, egocentric_user);
		prev_redirection = next_redirection;
		return next_redirection;
	}

	vec2f user_dir = rad_2_vec(egocentric_user->state.get_phys_heading());
	vec2f user_to_target = normalize(steer_target - egocentric_user->state.get_phys_pos());
	float angle_to_target = signed_angle(user_dir, user_to_target);

	// Need to update opaque vis polygon because we use it to check if the visibility 
	// event is associated with an event in the opaque vis polygon (which is the one 
	// we actually care about).
	update_visibility_polygons(egocentric_user); 

	// Determine if we need to do some special steering towards new the steering target.
	// If we have not gotten the user to look at the steer target yet, check if we 
	// have done it on the most recent frame. We only do this when aligned_with_slice_recently == false 
	// because after we successfully align the user with the steering target, we 
	// simply steer by ARC (until the next visibility event), so we assume that 
	// steering by ARC will be better than steering to the target point.
	if (!aligned_with_slice_recently) {
		aligned_with_slice_recently = math::abs(angle_to_target) < STEERING_ANGLE_THRESHOLD;
	}
	
	// Check for a visibility event
	bool visibility_event = phys_trans_view.update(egocentric_user->get_phys_pos(), phys_vis_poly) || virt_trans_view.update(egocentric_user->get_virt_pos(), virt_vis_poly);

	// If there is a visibility event, update the steering target (according to 
	// the new best slice). In theory, this should usually be the case whenever
	// there is a visibility event (since the view has likely changed a lot).
	// OR, if we just finished doing a reset maneuver, also treat as a visibility event.
	if (visibility_event || egocentric_user->state.just_finished_reset) {
		if (!egocentric_user->state.just_finished_reset) {
			int t = 4;
		}
		set_steer_target(sim_state, egocentric_user);
		steer_by_slice = math::abs(angle_to_target) >= STEERING_ANGLE_THRESHOLD;
		aligned_with_slice_recently = steer_by_slice;
	}
	// No new visibility event recently
	else {
		// Have not yet steered the user to the visibility slice target, 
		// keep steering them towards it.
		if (!aligned_with_slice_recently) {
			steer_by_slice = true;
		}
		// Got the user to face the visibility slice target, so now we can steer
		// by regular ARC.
		else {
			steer_by_slice = false;
		}
	}

	// Set the gains accordingly
	compute_proximities(sim_state, egocentric_user);
	redirection_unit next_redirection = set_gains(dx, dy, dtheta, sim_state, egocentric_user);
	prev_redirection = next_redirection;
	return next_redirection;
}

redirection_unit arc::set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	redirection_unit redir_unit;

	timestep::num_timesteps;
	// Steer the user towards the best slice.
	if (steer_by_slice) {
		redir_unit = get_visibility_slice_gains(dx, dy, dtheta, sim_state, egocentric_user);
	}
	// Steer by normal ARC because the user is now facing the best-matching slice.
	else {
		redir_unit = get_proximity_steering_gains(dx, dy, dtheta, sim_state, egocentric_user);
	}

	return redir_unit;
}

void arc::compute_proximities(simulation_state& sim_state, user* egocentric_user) {
	vec2f heading = rad_2_vec(egocentric_user->state.get_phys_heading());
	std::vector<vec2f> directions{ heading,						 // Forward
								   vec2f(heading.y, -heading.x), // Left
								   vec2f(-heading.y, heading.x)  // Right
	};

	float phys_distance_north, phys_distance_east, phys_distance_west;
	environment* phys_env = (environment*)egocentric_user->physical_env();
	vec2f phys_pos = egocentric_user->state.get_phys_pos();
	for (int i = 0; i < directions.size(); i++) {
		float closest = math::max_float;
		vec2f* p1;
		vec2f* p2;
		for (wall* w : phys_env->get_walls()) {
			p1 = w->get_vertices()[0];
			p2 = w->get_vertices()[1];
			float t = geom::ray_line_intersect(&phys_pos, &(directions[i]), p1, p2);
			if (t >= 0 && t < closest) {
				closest = t;
			}
		}
		for (obstacle* o : phys_env->get_obstacles()) {
			for (wall* w : o->get_walls()) {
				p1 = w->get_vertices()[0];
				p2 = w->get_vertices()[1];
				float t = geom::ray_line_intersect(&phys_pos, &(directions[i]), p1, p2);
				if (t >= 0 && t < closest) {
					closest = t;
				}
			}
		}

		// Assign the distance value to the correct direction.
		switch (i) {
		case 0: {
			phys_distance_north = closest;
			break;
		}
		case 1: {
			phys_distance_east = closest;
			break;
		}
		case 2: {
			phys_distance_west = closest;
			break;
		}
		}
	}

	float virt_distance_north, virt_distance_east, virt_distance_west;
	environment* virt_env = (environment*)egocentric_user->virtual_env();
	vec2f virt_pos = egocentric_user->state.get_virt_pos();
	for (int i = 0; i < directions.size(); i++) {
		float closest = math::max_float;
		vec2f* p1;
		vec2f* p2;
		for (wall* w : virt_env->get_walls()) {
			p1 = w->get_vertices()[0];
			p2 = w->get_vertices()[1];
			float t = geom::ray_line_intersect(&virt_pos, &(directions[i]), p1, p2);
			if (t >= 0 && t < closest) {
				closest = t;
			}
		}
		for (obstacle* o : virt_env->get_obstacles()) {
			for (wall* w : o->get_walls()) {
				p1 = w->get_vertices()[0];
				p2 = w->get_vertices()[1];
				float t = geom::ray_line_intersect(&virt_pos, &(directions[i]), p1, p2);
				if (t >= 0 && t < closest) {
					closest = t;
				}
			}
		}

		// Assign the distance value to the correct direction.
		switch (i) {
		case 0: {
			virt_distance_north = closest;
			break;
		}
		case 1: {
			virt_distance_east = closest;
			break;
		}
		case 2: {
			virt_distance_west = closest;
			break;
		}
		}
	}

	phys_dist_north = phys_distance_north;
	virt_dist_north = virt_distance_north;
	cur_north_loss = phys_distance_north - virt_distance_north;
	cur_east_loss = phys_distance_east - virt_distance_east;
	cur_west_loss = phys_distance_west - virt_distance_west;
	cur_alignment = std::vector<float>{ cur_north_loss, cur_east_loss, cur_west_loss };
}

void arc::set_steer_target(simulation_state& sim_state, user* egocentric_user) {
	slice* virt_slice = virt_vis_poly->slices[0];
	float best_diff = math::max_float;

	for (slice* s : phys_vis_poly->slices) {
		if (phys_vis_poly->slices[0]->theta_offset <= math::pi * 0.5f) {
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

	steer_target = best_slice->target_pt; // The position in world space that is the "end" of the slice.
}

float arc::get_symm_diff_area(slice* phys_slice, slice* virt_slice) {
	vec2f origin = vec2f(0.0f, 0.0f);
	Polygon_2 phys_p;
	//for (vec2f* v : *(phys_slice->pts)) {
	for (vec2f* v : phys_slice->pts) {
		float offset = signed_angle(phys_slice->bisector, vec2f(1.0f, 0.0f));
		vec2f rotated = rotate_around(origin, *v, offset);
		phys_p.push_back(Point_2(rotated.x, rotated.y));
	}

	Polygon_2 virt_p;
	//for (vec2f* v : *(virt_slice->pts)) {
	for (vec2f* v : virt_slice->pts) {
		float offset = signed_angle(virt_slice->bisector, vec2f(1.0f, 0.0f));
		vec2f rotated = rotate_around(origin, *v, offset);
		virt_p.push_back(Point_2(rotated.x, rotated.y));
	}

	Pwh_list_2 symmR;
	CGAL::symmetric_difference(phys_p, virt_p, std::back_inserter(symmR));
	float outer_area = 0.0f;
	float inner_area = 0.0f;
	
	for (auto it = symmR.begin(); it != symmR.end(); ++it) {
		auto test = (*it).outer_boundary();
		auto test2 = test.area();
		outer_area += (float)to_double(test2);

		int k = 1;
		for (auto hit = (*it).holes_begin(); hit != (*it).holes_end(); ++hit, ++k){
			std::vector<vec2f*> pts;
			for (auto vit = (*hit).vertices_begin(); vit != (*hit).vertices_end(); ++vit) {
				pts.insert(pts.begin(), new vec2f((float)to_double((*vit).x()),
					(float)to_double((*vit).y())));
			}
			inner_area += geom::polygon_area(pts);
		}
	}

	return outer_area - inner_area;
}

void arc::update_loss(simulation_state& sim_state, user* egocentric_user) {
	time_t start_time;
	start_time = time(NULL);
	cur_north_loss = 0.0f;
	cur_east_loss = 0.0f;
	cur_west_loss = 0.0f;

	cur_loss = phys_vis_poly - virt_vis_poly;

	time_t end_time;
	end_time = time(NULL);
	update_loss_time += end_time - start_time;
}

float arc::get_proximity_in_visibility_polygon(float dir, std::vector<vec2f> poly) {
	time_t start_time;
	start_time = time(NULL);

	float best_dist = math::max_float;
	vec2f vec_dir = rad_2_vec(dir);
	
	for (int i = 0; i < poly.size(); i++) {
		vec2f p1 = poly[i];
		vec2f p2 = poly[(i+1)%poly.size()];
		float d = geom::ray_line_intersect(new vec2f(0.0f, 0.0f), &vec_dir, &p1, &p2); // Ray origin is (0, 0) because the visibility polygon has been recentered.
		if (d != -1.0f && d < best_dist) best_dist = d;
	}

	time_t end_time;
	end_time = time(NULL);
	get_proximity_in_visibility_polygon_time += end_time - start_time;
	return best_dist;
}

/**
 * This is the normal steering by alignment (ARC), from the TVCG 2021 paper by Williams et al.
*/
redirection_unit arc::get_proximity_steering_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	redirection_unit redir_unit;
	cur_loss = math::abs(cur_north_loss) + math::abs(cur_east_loss) + math::abs(cur_west_loss);
	if (cur_loss < REDIRECTION_LOSS_THRESHOLD ||
		timestep::num_timesteps == 0) {
		cur_rota_gain = 1.0f;
		cur_trans_gain = 1.0f;
		cur_curve_gain = 0.0f;
		apply_rota = false;
		apply_trans = false;
		apply_curve = false;
		return redir_unit;
	}

	if (dtheta) {
		float loss_difference = cur_loss - prev_loss;
		// User is turning in a way that makes the alignment worse. Slow down their turning.
		if (cur_loss > prev_loss) {
			redir_unit.rota_gain = ((1.0f - ROTA_GAIN_SMOOTHING) * prev_rota_gain) + (min_rota_gain * ROTA_GAIN_SMOOTHING);
			//redir_unit.rota_gain = 1.0f;
		}
		// User is turning in a way that improves the alignment. Speed up their turning.
		else {
			redir_unit.rota_gain = ((1.0f - ROTA_GAIN_SMOOTHING) * prev_rota_gain) + (max_rota_gain * ROTA_GAIN_SMOOTHING);
			//redir_unit.rota_gain = 1.0f;
		}

		redir_unit.apply_rota = true;
		redir_unit.apply_trans = false;
		redir_unit.apply_curve = false;
		redir_unit.apply_bend = false;
	}
	else if (dx || dy) {
		// The space to the left of the user is more misaligned than the 
		// space to the right of the user. Turn them to the left.
		if (cur_west_loss > cur_east_loss) {
			//redir_unit.curve_gain = cur_curve_per_deg;
			redir_unit.curve_gain = set_curvature_gain(CURVE_TO_LEFT);
			redir_unit.curve_gain_dir = CURVE_TO_LEFT;
			redir_unit.apply_curve = true;
		}
		// The space to the right of the user is more misaligned than the 
		// space to the right of the user. Turn them to the right.
		else if (cur_east_loss > cur_west_loss) {
			//redir_unit.curve_gain = cur_curve_per_deg;
			redir_unit.curve_gain = set_curvature_gain(CURVE_TO_RIGHT);
			redir_unit.curve_gain_dir = CURVE_TO_RIGHT;
			redir_unit.apply_curve = true;
		}
		// The spaces on both sides of the user are equally aligned, so don't turn them.
		else {
			redir_unit.apply_curve = false;
		}

		// There is more physical space in front of the user than there is virtual 
		// space in front of the user, so make them walk faster in real world.
		if (cur_north_loss > 0.0f) {
			//redir_unit.trans_gain = max_trans_gain;
			redir_unit.trans_gain = set_translation_gain(sim_state, egocentric_user);
			redir_unit.apply_trans = true;
		}
		else if (cur_north_loss < 0.0f) {
			//redir_unit.trans_gain = min_trans_gain;
			redir_unit.trans_gain = set_translation_gain(sim_state, egocentric_user);
			redir_unit.apply_trans = true;
		}
		else {
			redir_unit.trans_gain = 1.0f;
			redir_unit.apply_trans = false;
		}

		redir_unit.apply_rota = false;
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

redirection_unit arc::get_visibility_slice_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user) {
	redirection_unit redir_unit;
	vec2f user_dir = rad_2_vec(egocentric_user->state.get_phys_heading());
	vec2f user_to_target = normalize(steer_target - egocentric_user->state.get_phys_pos());
	float angle_to_target = signed_angle(user_dir, user_to_target);

	if (dtheta) {
		// User has to turn right to face the gradient
		if (math::sign(angle_to_target) == -1) {
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
		if (math::sign(angle_to_target) == -1) {
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
		// the user in the PE relative to the VE. This is the same as ARC.
		// There is more physical space in front of the user than there is virtual 
		// space in front of the user, so make them walk faster in real world.
		if (cur_north_loss > 0.0f) {
			//redir_unit.trans_gain = max_trans_gain;
			redir_unit.trans_gain = set_translation_gain(sim_state, egocentric_user);
			redir_unit.apply_trans = true;
		}
		else if (cur_north_loss < 0.0f) {
			//redir_unit.trans_gain = min_trans_gain;
			redir_unit.trans_gain = set_translation_gain(sim_state, egocentric_user);
			redir_unit.apply_trans = true;
		}
		else {
			redir_unit.trans_gain = 1.0f;
			redir_unit.apply_trans = false;
		}

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

float arc::set_rotation_gain(simulation_state& sim_state, user* egocentric_user) {
	//todo
	//float virt_distance_north = cur_virt_vis_poly.distance_north;
	//float phys_distance_north = cur_phys_vis_poly.distance_north;

	//float trans_gain = p/*hys_distance_north / virt_distance_north;
	//trans_gain = math::clamp(trans_gain, min_trans_gain, max_trans_gain);
	//return trans_gain;*/
	return 0.0f;
}

float arc::set_translation_gain(simulation_state& sim_state, user* egocentric_user) {

	float trans_gain = phys_dist_north / virt_dist_north;
	trans_gain = math::clamp(trans_gain, min_trans_gain, max_trans_gain);

	//if (trans_gain < 1.0f) trans_gain = min_trans_gain;
	//if (trans_gain > 1.0f) trans_gain = max_trans_gain;
	return trans_gain;
}

float arc::set_curvature_gain(int direction) {
	float scale;
	if (direction == CURVE_TO_LEFT) {
		scale = math::min(1.0f, cur_west_loss);
	}
	else {
		scale = math::min(1.0f, cur_east_loss);
	}
	return math::min(cur_curve_per_deg * math::abs(scale), cur_curve_per_deg);

	/*
	float min_val = math::min(cur_west_loss, cur_east_loss);
	float max_val = math::max(cur_west_loss, cur_east_loss);

	// Avoid division by zero
	if (min_val) {
		float ratio = (math::abs(max_val) / math::abs(min_val)) - 1.0f;
		return math::min(cur_curve_per_deg * ratio, cur_curve_per_deg);
	}
	else {
		return cur_curve_per_deg;
	}
	*/
}

float arc::distance_to_closest_feature(vec2f pos, vec2f dir, visibility_polygon vis_poly) {
	time_t start_time;
	start_time = time(NULL);

	float min_dist = math::max_float;
	for (int i = 0; i < vis_poly.verts.size(); i++) {
		vec2f* p1 = vis_poly.verts[i % vis_poly.verts.size()];
		vec2f* p2 = vis_poly.verts[(i + 1) % vis_poly.verts.size()];
		float t = geom::ray_line_intersect(&(vec2f(0.0f, 0.0f)), &dir, p1, p2);
		if (t >= 0.0f && t < min_dist) {
			min_dist = t;
		}
	}

	time_t end_time;
	end_time = time(NULL);
	distance_to_closest_feature_time += end_time - start_time;
	return min_dist;
}

std::vector<std::vector<vec2f>> arc::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
	std::vector<std::vector<vec2f>> test;
	return test;
}

void arc::reset_to_gradient(vec2f phys_heading, float virt_heading, std::vector<trajectory_unit>* path, std::deque<proximity_container*> prox_queue, physical_environment* phys_env, vec2f phys_pos, vec2f virt_pos) {
	time_t start_time;
	start_time = time(NULL);

	gradient_dir = compute_gradient(phys_env, prox_queue, phys_pos);
	reorient_to_target(phys_heading, virt_heading, phys_pos, virt_pos, gradient_dir + phys_pos, path);

	time_t end_time;
	end_time = time(NULL);
	reset_to_gradient_time += end_time - start_time;
}

vec2f arc::compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos) {
	time_t start_time;
	start_time = time(NULL);

	std::vector<object*> objects;
	for (proximity_container* c : prox_queue) {
		if (c->get_obstacle()->space == object::SPACE_TYPE::PHYS) {
			objects.push_back(c->get_obstacle());
		}
	}
	float cur_repulsive_force = compute_repulsive_force(phys_pos, objects);

	float min_force = std::numeric_limits<float>::max();
	vec2f min_force_pos;
	for (float theta : theta_values) {
		vec2f new_pos = phys_pos + (rad_2_vec(theta) * SFR2G_STEP_SIZE);
		if (!phys_env->point_is_legal(new_pos)) continue;

		float new_force = compute_repulsive_force(new_pos, objects);
		if (new_force <= min_force) {
			min_force = new_force;
			min_force_pos = new_pos;
		}
	}

	assert(min_force != std::numeric_limits<float>::max());

	time_t end_time;
	end_time = time(NULL);
	compute_gradient_time += end_time - start_time;
	return normalize(min_force_pos - phys_pos);
}

float arc::compute_repulsive_force(vec2f pos, std::vector<object*> objects) {
	time_t start_time;
	start_time = time(NULL);

	float dist_sum = 0.0f;
	for (object* obj : objects) {
		if (obj->space == object::SPACE_TYPE::PHYS)
			dist_sum += 1.0f / obj->distance(pos);
	}

	time_t end_time;
	end_time = time(NULL);
	compute_repulsive_force_time += end_time - start_time;
	return dist_sum;
}