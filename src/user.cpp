#include <iostream>
#include <algorithm>
#include <string>
#include <chrono>

#include "user.hpp"
#include "timestep.h"
#include "geometry.h"
#include "config.h"

user::user() {

}

user::user(int id, vec2f phys_start_pos, vec2f virt_start_pos, float phys_heading, float virt_heading, int num_paths, int num_waypoints, motion_model::PATH_MODEL path_model, motion_model::TRAJECTORY_MODEL trajectory_model, redirector* rdw, environment* phys_env, environment* virt_env) {
    type = obstacle::OBJECT_TYPE::USER;
    space = obstacle::SPACE_TYPE::PHYS; // TODO: is this wrong? do I even care about this value?

    this->radius = config::USER_RADIUS;

    init_state(phys_env, virt_env, phys_start_pos, virt_start_pos, phys_heading, virt_heading, num_paths, num_waypoints, path_model, trajectory_model, rdw);

    this->proximity_queue = std::deque<proximity_container*>();
    this->add_phys_env((physical_environment*)phys_env);
    this->add_virt_env((virtual_environment*)virt_env);
    this->rdw = rdw;
    this->num_collisions = 0;
    this->id = id;
    this->paths_complete = 0;

    num_vertices = 4;
    float draw_rad = (radius == 0.0f) ? config::DISTANCE_THRESHOLD : radius;
    base_verts.push_back(vec2f(draw_rad, 0.0f)); // nose
    base_verts.push_back(draw_rad * normalize(vec2f(-draw_rad, draw_rad))); // left corner
    base_verts.push_back(vec2f(-0.25f * draw_rad, 0.0f)); // back
    base_verts.push_back(draw_rad * normalize(vec2f(-draw_rad, -draw_rad))); // right corner
    gl_indices.push_back(0);
    gl_indices.push_back(1);
    gl_indices.push_back(2);
    gl_indices.push_back(0);
    gl_indices.push_back(2);
    gl_indices.push_back(3);

    this->done = false;
    this->finished_path = false;
    user_history.log(state);
}

void user::init_state(environment* phys_env, environment* virt_env, vec2f phys_start_pos, vec2f virt_start_pos, float phys_heading, float virt_heading, int num_paths, int num_waypoints, motion_model::PATH_MODEL path_model, motion_model::TRAJECTORY_MODEL trajectory_model, redirector* rdw) {
    vec2f final_virt_pos;
    vec2f final_phys_pos;
    float final_virt_heading;
    float final_phys_heading;

    // Virtual position
    if (config::RANDOM_VIRT_START_POS) {
        vec2f rand_virt_start_pos = virt_env->sample_point();
        final_virt_pos = rand_virt_start_pos;
    }
    else {
        final_virt_pos = virt_start_pos;
    }
    // Physical position
    if (config::MATCH_PHYS_VIRT_POS) {
        final_phys_pos = final_virt_pos;
    }
    else if (config::RANDOM_PHYS_START_POS) {
        vec2f rand_phys_start_pos = phys_env->sample_point();
        final_phys_pos = rand_phys_start_pos;
    }
    else if (config::RADIUS_PHYS_START_POS) {
        vec2f rand_phys_start_pos = phys_env->sample_point();
        float dist_to_virt_pos = length(final_virt_pos - rand_phys_start_pos);

        while (dist_to_virt_pos > config::RADIUS_PHYS_START_POS_THRESHOLD) {
            rand_phys_start_pos = phys_env->sample_point();
            dist_to_virt_pos = length(final_virt_pos - rand_phys_start_pos);
        }

        final_phys_pos = rand_phys_start_pos;
    }
    else {
        final_phys_pos = phys_start_pos;
    }

    // Reset heading
    // Virtual heading
    if (config::RANDOM_VIRT_HEADING) {
        float rand_virt_heading = 0.0f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (math::two_pi - 0.0f)));
        final_virt_heading = rand_virt_heading;
    }
    else {
        final_virt_heading = virt_heading;
    }
    // Physical heading
    if (config::MATCH_PHYS_VIRT_HEADING) {
        final_phys_heading = final_virt_heading;
    }
    else {
        if (config::RANDOM_PHYS_HEADING) {
            float rand_phys_heading = 0.0f + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (math::two_pi - 0.0f)));
            final_phys_heading = rand_phys_heading;
        }
        else {
            final_phys_heading = phys_heading;
        }
    }

    this->state = user_state(final_phys_pos, final_virt_pos, final_phys_heading, final_virt_heading, num_paths, num_waypoints, path_model, trajectory_model, rdw);
}

std::vector<std::vector<vec2f>> user::get_gradient_data() {
    return rdw->get_gradient_data(phys_env, proximity_queue);
}

int user::vertex_buffer_size() {
    return base_verts.size() * 3 * sizeof(float);
}

int user::index_buffer_size() {
    return gl_indices.size() * sizeof(unsigned int);
}

void user::step(simulation_state& sim_state){
    done = state.done;
    int t = timestep::num_timesteps;

    if (!finished_path) {
        //handle_collisions(sim_state);
        state.step(sim_state, this);
        handle_collisions(sim_state);
        user_history.log(state); 
    }
    finished_path = state.path.size() == 0;
    if (this->paths_complete < state.paths_complete && finished_path) {
        paths_complete++;
        finished_path = true;
    }
}

void user::prep_for_next_path(simulation_state& sim_state) {
    write(0);
    reset_state(phys_env, sim_state.virt_env);
    user_history.log(state); // Record the initial state.

    if (paths_complete == config::NUM_PATHS) {
        state.done = true;
        done = true;
        finished_path = true;
    }
    else {
        generate_motion(sim_state.virt_env);
    }
}

void user::write(int paths_increment) {
    user_history.write(this, paths_complete + paths_increment);
}

void user::reset_state(environment* phys_env, environment* virt_env) {
    std::cout << "Path #" << paths_complete << " finished. Resetting..." << std::endl;
    std::cout << "Num pts remaining in path: " << state.path.size() << std::endl;

    state.reset_state(phys_env, virt_env);
    user_history.reset();
}

bool sort_queue(const proximity_container* c1, const proximity_container* c2) {
    return c1->get_distance() < c2->get_distance();
}

void user::broadcast_pause() {
    int wait_timer = rdw->reset_policy->reset_timer;
    for (obstacle* o : phys_env->get_obstacles()) {
        if (o->is_dynamic()) {
            o->wait_for_reset(wait_timer);
        }
    }
    for (obstacle* o : virt_env->get_obstacles()) {
        if (o->is_dynamic()) {
            o->wait_for_reset(wait_timer);
        }
    }
}

void user::handle_collisions(simulation_state& sim_state) {
    std::deque<proximity_container*>::iterator pos;
    for (pos = proximity_queue.begin(); pos != proximity_queue.end(); ++pos) {
        (*pos)->update_distance();
    }

    sort(proximity_queue.begin(), proximity_queue.end(), sort_queue);

    if (trigger_reset) {
        state.reset(sim_state, this);
        num_collisions++;
        trigger_reset = false;
        broadcast_pause();

        if (get_closest_obstacle(object::SPACE_TYPE::PHYS)->get_obstacle()->mvmt == MOVEMENT_TYPE::DYNAMIC) {
            state.post_dynamic_obstacle_reset_timer = 10;
        }

        return;
    }

    // TODO: it isn't clear when others in the literature initiate the reset. is it considering the radius of the user, or just their center position?? need to re-read literature and verify this.
    proximity_container* closest_obj = get_closest_obstacle(object::SPACE_TYPE::PHYS);

    /*
    if (math::abs(closest_obj->get_distance()) <= config::RESET_DISTANCE_CHECK_VALUE &&
        state.nav_state != user_state::NAVIGATION_STATE::RESETTING) {
        trigger_reset = true;
        return;
    }s
    */
    float ddd = math::abs(closest_obj->get_distance());
    if (math::abs(closest_obj->get_distance()) <= config::RESET_DISTANCE_CHECK_VALUE &&
        state.nav_state != user_state::NAVIGATION_STATE::RESETTING && 
        state.post_dynamic_obstacle_reset_timer == 0) {
        // Check if the close object is in front of the user (they will collide if it's in front)
        std::vector<vec2f*> closest_feature = closest_obj->get_closest_feature();

        // Closest object is a user
        if (closest_feature.size() == 1) {
            if (paths_complete > 43) {
                std::cout << "collided with a user" << std::endl;
            }
            //vec2f cur_pos = state.get_phys_pos();
            //// Check if the user is facing the other user.
            //vec2f cur_phys_heading = rad_2_vec(state.get_phys_heading());
            //vec2f s1 = cur_pos + vec2f(-cur_phys_heading.y, cur_phys_heading.x);
            //vec2f s2 = cur_pos + vec2f(cur_phys_heading.y, -cur_phys_heading.x);

            //if (geom::orient(s1, s2, closest_feature[0]) != 1) return; // Not facing

            //vec2f next_pos = state.get_next_pos();
            //user* other_user = (user*)proximity_queue.front()->get_obstacle();
            //vec2f other_user_cur_pos = other_user->state.get_phys_pos();
            //vec2f other_user_next_pos = other_user->state.get_next_pos();
            //float intersect_time = geom::circle_circle_intersect(cur_pos, next_pos, other_user_cur_pos, other_user_next_pos, radius, other_user->radius);

            //// Keep walking if we won't collide in the next timestep
            //if (intersect_time == -1.0f || intersect_time > 1.0f) return;

            // Adjust path to walk until we collide, then incur reset, then finish the path.
            //step_until_collision(intersect_time);

            user* other_user = (user*)proximity_queue.front()->get_obstacle();
            vec2f cur_heading = rad_2_vec(state.get_phys_heading());
            vec2f other_user_heading = rad_2_vec(other_user->state.get_phys_heading());
            // Facing each other
            if (dot(cur_heading, other_user_heading) < 0.0f) {
                other_user->handle_collisions(sim_state);
                trigger_reset = true;
            }
        }
        // Closest object is an obstacle
        else {
            //vec2f s1 = closest_feature[0];
            //vec2f s2 = closest_feature[1];

            //std::cout << "p1: " << s1 << std::endl;
            //std::cout << "p2: " << s2 << std::endl;
            //std::cout << "======" << std::endl;

            //// Check if the user is facing the closest feature.
            //vec2f cur_pos = state.get_phys_pos();
            //vec2f cur_phys_heading = rad_2_vec(state.get_phys_heading());
            //vec2f temp1 = state.get_phys_pos() + vec2f(-cur_phys_heading.y, cur_phys_heading.x);
            //vec2f temp2 = state.get_phys_pos() + vec2f(cur_phys_heading.y, -cur_phys_heading.x);
            //if (geom::orient(temp1, temp2, closest_feature[0]) != 1 &&
            //    geom::orient(temp1, temp2, closest_feature[1]) != 1) return; // Not facing
            //
            ////the missed ray shoot is on frame 29538. so i need to stop my debug at 29537
            //// User is going to collide with the feature
            //vec2f next_pos = state.get_next_pos();
            //float intersect_time = geom::line_circle_intersect(s1, s2, cur_pos, next_pos, radius);

            //// Keep walking if we won't collide in the next timestep
            //if (intersect_time == -1.0f) return;

            // Adjust path to walk until we collide, then incur reset, then finish the path.
            //step_until_collision(intersect_time);

            vec2f* s1 = closest_feature[0];
            vec2f* s2 = closest_feature[1];
            vec2f cur_heading = rad_2_vec(state.get_phys_heading());
            /*
            vec2f feature_dir = normalize(s2 - s1);
            vec2f normal1 = vec2f(-feature_dir.y, feature_dir.x);
            vec2f normal2 = vec2f(feature_dir.y, -feature_dir.x);
            vec2f feature_to_user = normalize(state.get_phys_pos() - s1);
            vec2f true_normal;
            // User is on the side of the feature pointed to by normal1
            if (dot(feature_to_user, normal1) > 0.0f) {
                true_normal = normal1;
            }
            else {
                true_normal = normal2;
            }
            */

            if (proximity_queue.front()->get_obstacle()->is_dynamic()) {
                float dist = proximity_queue.front()->get_obstacle()->distance(state.get_phys_pos());
                float d2 = math::abs(closest_obj->get_distance());
                int eruioth = 4;
            }

            float t = geom::ray_line_intersect(new vec2f(state.get_phys_pos()), new vec2f(rad_2_vec(state.get_phys_heading())), s1, s2);

            vec2f true_normal = geom::normal_to_point(s1, s2, new vec2f(state.get_phys_pos()));
            // Facing each other or collided with a dynamic obstacle
            if (dot(cur_heading, true_normal) < 0.0f ||
                proximity_queue.front()->get_obstacle()->is_dynamic()) {
            //if (t >= 0.0f) {
                trigger_reset = true;
            }
        }
    }
}

void user::trigger_reset_on_next_step() {
    trigger_reset = true;
}

// Currently unused because it doesn't work properly at guaranteeing we catch all collisions. date of writing this ocmment: Aug 12, 2020 1:29 AM, but it has been unused for much longer.
void user::step_until_collision(float t) {
    assert(t >= 0.0f);
    trajectory_unit next_move_first_fraction = state.path[0] * t;
    trajectory_unit next_move_second_fraction = state.path[0] * (1.0f - t);

    state.path.erase(state.path.begin());
    auto it = state.path.insert(state.path.begin(), next_move_second_fraction);
    state.path.insert(it, next_move_first_fraction);
    trigger_reset = true;
}

void user::print_pos() {
    std::cout << "(" << state.get_phys_pos().x << ", " << state.get_phys_pos().y << ")" << std::endl;
}

void user::add_phys_env(physical_environment* phys_env) {
    for (wall* wall : phys_env->get_walls()) {
        proximity_queue.push_back(new proximity_container(this, wall));
    }
    for (obstacle* obstacle : phys_env->get_obstacles()) {
        proximity_queue.push_back(new proximity_container(this, obstacle));
    }
    this->phys_env = phys_env;
}

void user::add_virt_env(virtual_environment* virt_env) {
    for (wall* wall : virt_env->get_walls()) {
        proximity_queue.push_back(new proximity_container(this, wall));
    }
    for (obstacle* obstacle : virt_env->get_obstacles()) {
        proximity_queue.push_back(new proximity_container(this, obstacle));
    }
    this->virt_env = virt_env;
}

void user::add_other_users(std::vector<user*> users) {
    for (user* user : users) {
        if (user != this) {
            proximity_queue.push_back(new proximity_container(this, user));
            other_users.push_back(user);
        }
    }
}

std::vector<vec2f*> user::get_vertices() {
    vec2f p = state.get_phys_pos();
    std::vector<vec2f*> temp;
    vec2f* temp2 = new vec2f(p.x, p.y);
    temp.push_back(temp2);
    return temp;
}

float user::distance(vec2f p) {
    //TODO: refactor distance function to have a flag for phys or virtual distance?? right now i only care about the phys proximity, since the virt env is always an empty plane... prob do want to refactor it eventually.
    return length(p - state.get_phys_pos());
}

void user::generate_motion(virtual_environment* virt_env) {
    state.generate_motion(virt_env);
    finished_path = false;
}

vec2f user::get_phys_pos() {
    return state.get_phys_pos();
}

physical_environment* user::get_phys_env() {
    return phys_env;
}

vec2f user::get_virt_pos() {
    return state.get_virt_pos();
}

char* user::get_redirector_name() {
    return rdw->name;
}

char* user::get_resetter_name() {
    return rdw->reset_policy->name;
}

std::vector<vec2f> user::get_draw_vertices(float alpha) {
    // Copy verts to a new vector (list)
    std::vector<vec2f> temp;
    for (vec2f p : base_verts) {
        temp.push_back(vec2f(p));
    }

    vec2f interpolated_pos = vec2f(state.get_phys_pos().x, state.get_phys_pos().y);
    float interpolated_heading = state.get_phys_heading();
    trajectory_unit prev_move = trajectory_unit(state.get_prev_move().x, state.get_prev_move().y, state.get_prev_move().theta);

    vec2f prev_move_vec = vec2f(math::cos(state.get_phys_heading()) * prev_move.x,
        math::sin(state.get_phys_heading()) * prev_move.y);
    interpolated_pos += prev_move_vec * state.get_velocity() * timestep::dt * alpha;
    interpolated_heading += math::radians(state.get_angular_vel()) * timestep::dt * alpha * prev_move.theta;

    for (int i = 0; i < temp.size(); i++) {
        // Roate the arrow according to the heading
        float new_x = temp[i].x * math::cos(interpolated_heading) - temp[i].y * math::sin(interpolated_heading);
        float new_y = temp[i].y * math::cos(interpolated_heading) + temp[i].x * math::sin(interpolated_heading);
        temp[i].x = new_x;
        temp[i].y = new_y;
        // Translate the points to the current position
        temp[i].x += interpolated_pos[0];
        temp[i].y += interpolated_pos[1];
    }

    return temp;
}

bool user::is_blocking_path(vec2f start, vec2f end, float radius) {
    vec2f path = end - start;
    vec2f perpendicular_offset = normalize(vec2f(-path.y, path.x)) * radius;
    float dist = geom::line_point_distance(new vec2f(start), new vec2f(end), new vec2f(state.get_phys_pos()));
    return (dist < radius);
}

/**
 * Here, the "walls" are a bounding box around the user.
 */
vec2f user::get_closest_wall(vec2f p) {
    vec2f heading = rad_2_vec(state.get_phys_heading());
    vec2f heading_perpendicular = vec2f(-heading.y, heading.x);
    vec2f phys_pos = state.get_phys_pos();

    std::vector<vec2f> bb;
    bb.push_back(phys_pos + (radius * heading) + (radius * heading_perpendicular));
    bb.push_back(phys_pos - (radius * heading) + (radius * heading_perpendicular));
    bb.push_back(phys_pos + (radius * heading) - (radius * heading_perpendicular));
    bb.push_back(phys_pos - (radius * heading) - (radius * heading_perpendicular));

    float best_dist = std::numeric_limits<float>::max();
    vec2f best_wall;
    for (int i = 0; i < bb.size(); i++) {
        float dist = geom::line_point_distance(new vec2f(bb[i]), new vec2f(bb[(i+1) % bb.size()]), new vec2f(p));
        if (dist < best_dist) {
            best_dist = dist;
            best_wall = bb[(i + 1)] - bb[i];
        }
    }

    return normalize(best_wall);
}

proximity_container* user::get_closest_obstacle(object::SPACE_TYPE space_to_search) {
    float closest_dist = math::max_float;
    proximity_container* closest_object = NULL;

    for (proximity_container* o : proximity_queue) {
        if (o->get_obstacle()->space == space_to_search &&
            o->get_distance() < closest_dist) {
            closest_dist = o->get_distance();
            closest_object = o;
        }
    }

    assert(closest_object != NULL && "No closest object found! Is your environment empty?");
    return closest_object;
}

physical_environment* user::physical_env() {
    return phys_env;
}

virtual_environment* user::virtual_env() {
    return virt_env;
}

bool user::is_dynamic() {
    return mvmt == MOVEMENT_TYPE::DYNAMIC;
}

// // Python interface
// namespace py = pybind11;

// PYBIND11_MODULE(user, m) {
//     py::class_<User>(m, "User")
//         .def(py::init<>())
//         .def("add_user", &User::move_x)
//         .def("step", &User::print_pos);
// }