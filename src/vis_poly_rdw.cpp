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

#include "vis_poly_rdw.h"
#include "math.hpp"
#include "user.hpp"
#include "geometry.h"
#include "config.h"

//#include "visibility.hpp"
#include "vector2.hpp"

vis_poly_rdw::vis_poly_rdw(physical_environment* phys_env, virtual_environment* virt_env, resetter* _resetter) {
	time_t start_time;
	start_time = time(NULL);

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

	//============================
	// Gradient stuff
	for (int i = 0; i < GRADIENT_SAMPLE_RATE; i++) {
		theta_values.push_back(((2 * math::pi) / GRADIENT_SAMPLE_RATE) * i);
	}

	//init_vis_poly_data(phys_env, virt_env);

	time_t end_time;
	end_time = time(NULL);
	constructor_time += end_time - start_time;
}

void vis_poly_rdw::init_vis_poly_data(physical_environment* phys_env, virtual_environment* virt_env) {
	// insert geometry into the arrangement
	std::vector<Segment_2> verts = phys_env->get_cgal_verts();
	CGAL::insert_non_intersecting_curves(phys_arr_env, verts.begin(), verts.end());

	verts = virt_env->get_cgal_verts();
	CGAL::insert_non_intersecting_curves(virt_arr_env, verts.begin(), verts.end());
}

//visibility_polygon vis_poly_rdw::get_vis_poly(vec2f pos, Arrangement_2 arr_env) {
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
	time_t start_time;
	start_time = time(NULL);

	std::chrono::milliseconds start = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()
		);

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

	int t = end.count() - start.count();
	time_t end_time;
	end_time = time(NULL);
	update_visibility_polygons_time += end_time - start_time;
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
	time_t start_time;
	start_time = time(NULL);
	//using namespace std::chrono;
	std::chrono::milliseconds start = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()
		);

	//update_dynamic_obstacles(); // TODO: add support for dyunamic obstacles, and update the vertices of those obstalces in the environment objects
	update_visibility_polygons(egocentric_user);
	//orient_visibility_polygons(egocentric_user);
	//get_simple_visibility_polygons();
	set_steer_target(sim_state, egocentric_user);

	update_loss(sim_state, egocentric_user);
	cur_alignment = std::vector<float>{ cur_north_loss, cur_east_loss, cur_west_loss };

	redirection_unit next_redirection = set_gains(dx, dy, dtheta, sim_state, egocentric_user, new node(vec2f(0.0f, 0.0f)));

	prev_loss = cur_loss;

	time_t end_time;
	end_time = time(NULL);
	std::chrono::milliseconds end = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()
		);
	//update_time += end_time - start_time;
	update_time += end.count() - start.count();
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

float vis_poly_rdw::get_symm_diff_area(slice* phys_slice, slice* virt_slice) {
	vec2f origin = vec2f(0.0f, 0.0f);
	Polygon_2 phys_p;
	for (vec2f* v : *(phys_slice->pts)) {
		float offset = signed_angle(phys_slice->bisector, vec2f(1.0f, 0.0f));
		vec2f rotated = rotate_around(origin, *v, offset);
		phys_p.push_back(Point_2(rotated.x, rotated.y));
	}

	Polygon_2 virt_p;
	for (vec2f* v : *(virt_slice->pts)) {
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

void vis_poly_rdw::update_loss(simulation_state& sim_state, user* egocentric_user) {
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

float vis_poly_rdw::get_proximity_in_visibility_polygon(float dir, std::vector<vec2f> poly) {
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

redirection_unit vis_poly_rdw::set_gains(float dx, float dy, float dtheta, simulation_state& sim_state, user* egocentric_user, node* optimal_node) {
	time_t start_time;
	start_time = time(NULL);

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

	time_t end_time;
	end_time = time(NULL);
	set_gains_time += end_time - start_time;
	return redir_unit;
}

float vis_poly_rdw::set_rotation_gain(simulation_state& sim_state, user* egocentric_user) {
	//todo
	//float virt_distance_north = cur_virt_vis_poly.distance_north;
	//float phys_distance_north = cur_phys_vis_poly.distance_north;

	//float trans_gain = p/*hys_distance_north / virt_distance_north;
	//trans_gain = math::clamp(trans_gain, min_trans_gain, max_trans_gain);
	//return trans_gain;*/
	return 0.0f;
}

float vis_poly_rdw::set_translation_gain(simulation_state& sim_state, user* egocentric_user) {

	float trans_gain = phys_north_loss / virt_north_loss;
	trans_gain = math::clamp(trans_gain, min_trans_gain, max_trans_gain);

	//if (trans_gain < 1.0f) trans_gain = min_trans_gain;
	//if (trans_gain > 1.0f) trans_gain = max_trans_gain;
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

float vis_poly_rdw::distance_to_closest_feature(vec2f pos, vec2f dir, visibility_polygon vis_poly) {
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

std::vector<std::vector<vec2f>> vis_poly_rdw::get_gradient_data(physical_environment* phys_env, std::deque<proximity_container*> prox_queue) {
	std::vector<std::vector<vec2f>> test;
	return test;
}

void vis_poly_rdw::reset_to_gradient(vec2f phys_heading, float virt_heading, std::vector<trajectory_unit>* path, std::deque<proximity_container*> prox_queue, physical_environment* phys_env, vec2f phys_pos, vec2f virt_pos) {
	time_t start_time;
	start_time = time(NULL);

	gradient_dir = compute_gradient(phys_env, prox_queue, phys_pos);
	reorient_to_target(phys_heading, virt_heading, phys_pos, virt_pos, gradient_dir + phys_pos, path);

	time_t end_time;
	end_time = time(NULL);
	reset_to_gradient_time += end_time - start_time;
}

vec2f vis_poly_rdw::compute_gradient(physical_environment* phys_env, std::deque<proximity_container*> prox_queue, vec2f phys_pos) {
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

float vis_poly_rdw::compute_repulsive_force(vec2f pos, std::vector<object*> objects) {
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