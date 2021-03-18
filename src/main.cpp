#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stb_image.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/filesystem.h>
#include <learnopengl/shader_m.h>

#include <iostream>
#include <limits>
#include <time.h>
#include <signal.h>

#include "include/imgui/imgui.h"
#include "include/imgui/imgui_impl_glfw.h"
#include "include/imgui/imgui_impl_opengl3.h"

#include "virtual_environment.hpp"
#include "physical_environment.hpp"
#include "simulation.hpp"
#include "user.hpp"
#include "motion_model.h"
#include "timestep.h"
#include "apf_grad.h"
#include "config.h"
#include "arc.h"
#include "node.h"
//#include "vec2f.hpp"
//#include "geometry.hpp"

//#include <include/boost_1_73_0/boost/lambda/lambda.hpp>
//#include "include/boost_1_73_0/boost/lambda/lambda.hpp"
//#include <include/boost/lambda/lambda.hpp>
#include "include/boost/lambda/lambda.hpp"
//#include <include/lambda/lambda.hpp>
//#include "include/lambda/lambda.hpp"
//#include <libs/boost/lambda/lambda.hpp>

#define IM_ARRAYSIZE(buf) (sizeof(buf) / sizeof(*buf))

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>            // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>            // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>          // Initialize with gladLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING2)
#define GLFW_INCLUDE_NONE       // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/Binding.h>  // Initialize with glbinding::Binding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING3)
#define GLFW_INCLUDE_NONE       // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/glbinding.h>// Initialize with glbinding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window, simulation sim);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;
int EXIT_EARLY = 0;

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <list>
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;
#include "print_utils.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <iostream>

void test() {
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef CGAL::Triangulation_vertex_base_2<K> Vb;
	typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
	typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
	typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
	typedef CDT::Vertex_handle Vertex_handle;
	typedef CDT::Point Point;

	CDT cdt;
	/*Vertex_handle va = cdt.insert(Point(2, 0));
	Vertex_handle vb = cdt.insert(Point(0, 2));
	Vertex_handle vc = cdt.insert(Point(-2, 0));
	Vertex_handle vd = cdt.insert(Point(0, -2));*/
	/*cdt.insert_constraint(va, vb);
	cdt.insert_constraint(vb, vc);
	cdt.insert_constraint(vc, vd);
	cdt.insert_constraint(vd, va);*/
	Vertex_handle va = cdt.insert(Point(3, 3));
	Vertex_handle vb = cdt.insert(Point(-3, 3));
	Vertex_handle vc = cdt.insert(Point(-3, -3));
	Vertex_handle vd = cdt.insert(Point(3, -3));
	Vertex_handle va21 = cdt.insert(Point(3, 3));
	cdt.insert_constraint(va, vb);
	cdt.insert_constraint(vb, vc);
	cdt.insert_constraint(vc, vd);
	cdt.insert_constraint(vd, va21);

	Vertex_handle va1 = cdt.insert(Point(2, 2));
	Vertex_handle vb1 = cdt.insert(Point(-2, 2));
	Vertex_handle vc1 = cdt.insert(Point(-2, -2));
	Vertex_handle vd1 = cdt.insert(Point(2, -2));
	cdt.insert_constraint(va1, vb1);
	cdt.insert_constraint(vb1, vc1);
	cdt.insert_constraint(vc1, vd1);
	cdt.insert_constraint(vd1, va1);
	std::list<Point> list_of_seeds;
	list_of_seeds.push_back(Point(0, 0));
	//list_of_seeds.push_back(Point(2, 2));
	//list_of_seeds.push_back(Point(-2, 2));
	//list_of_seeds.push_back(Point(-2, -2));
	//list_of_seeds.push_back(Point(2, -2));
	std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;
	std::cout << "Meshing the domain..." << std::endl;
	CGAL::refine_Delaunay_mesh_2(cdt, list_of_seeds.begin(), list_of_seeds.end(),
		Criteria(), false);
	//# false means we DO NOT draw the seed
	//# true means we DO draw the seed
	std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;
	std::cout << "Number of finite faces: " << cdt.number_of_faces() << std::endl;
	int mesh_faces_counter = 0;
	for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
		fit != cdt.finite_faces_end(); ++fit)
	{
		if (fit->is_in_domain()) {
			++mesh_faces_counter;
			std::cout << cdt.triangle(fit) << std::endl;
		}
		else {
			//std::cout << cdt.triangle(fit) << std::endl;
		}
	}
	std::cout << "Number of faces in the mesh domain: " << mesh_faces_counter << std::endl;

	int t = 4;


}

int run_graphics() {
	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// glfw window creation
	// --------------------
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Simulated RDW", NULL, NULL);
	if (window == NULL) {
		std::cout << "Failed to create GLFW wincoutdow" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// Decide GL+GLSL versions
#if __APPLE__
	// GL 3.2 + GLSL 150
	const char* glsl_version = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
	// GL 3.0 + GLSL 130
	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	//glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

	// Initialize OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
	bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
	bool err = gladLoadGL() == 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING2)
	bool err = false;
	glbinding::Binding::initialize();
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING3)
	bool err = false;
	glbinding::initialize([](const char* name) { return (glbinding::ProcAddress)glfwGetProcAddress(name); });
#else
	bool err = false; // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to requires some form of initialization.
#endif
	if (err)
	{
		fprintf(stderr, "Failed to initialize OpenGL loader!\n");
		return 1;
	}

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// ImGui menu stuff
	bool draw_gradient = false;

	// build and compile our shader zprogram
	// ------------------------------------
	Shader ourShader("6.1.coordinate_systems.vs", "6.1.coordinate_systems.fs"); // TODO change these filenames

	//std::vector<physical_environment*> phys_envs;
	//phys_envs.push_back(&config::phys_env1);
	//phys_envs.push_back(&config::phys_env2);
	simulation sim = simulation(config::phys_envs, &config::virt_env, config::users);

	// set up vertex data (and buffer(s)) and configure vertex attributes
	 //------------------------------------------------------------------
	int num_users = config::users.size();

	std::vector<std::vector<vec2f>> grad_data;
	std::vector<float> vert_data; // size = 33600 
	std::vector<unsigned int> index_data;
	unsigned int vbo, vao, ebo;
	if (config::DEBUG) {
		// TODO: make this get called only when the "show gradient" button is checked, so that it doesnt make startup very slow when we start in debug mode
		//grad_data = sim.users[0]->get_gradient_data();
		//int idx1 = 0;
		//unsigned int index_count = 0;
		//for (float i = -4.9f; i < 5.0f; i += 0.1f) {
		//	std::vector<vec2f> vec;
		//	int idx2 = 0;
		//	for (float j = -4.9f; j < 5.0f; j += 0.1f) {
		//		vec2f p = vec2f(i, j);
		//		if (config::phys_env1.point_is_legal(p)) {
		//			vec2f p1 = p;
		//			vec2f p2 = grad_data[idx1][idx2];
		//			p2 = (p2 * 0.07f) + p;
		//			vert_data.push_back(p1.x);
		//			vert_data.push_back(p1.y);
		//			vert_data.push_back(0.0f);
		//			vert_data.push_back(p2.x);
		//			vert_data.push_back(p2.y);
		//			vert_data.push_back(0.0f);
		//			index_data.push_back(index_count++);
		//			index_data.push_back(index_count++);
		//		}
		//		idx2++;
		//	}
		//	idx1++;
		//}

		//glGenVertexArrays(1, &vao);
		//glGenBuffers(1, &vbo);
		//glGenBuffers(1, &ebo);
		//glBindVertexArray(vao);
		//glBindBuffer(GL_ARRAY_BUFFER, vbo);
		//glBufferData(GL_ARRAY_BUFFER, vert_data.size() * sizeof(float), &vert_data[0], GL_STATIC_DRAW);
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
		//glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_data.size() * sizeof(unsigned int), &index_data[0], GL_STATIC_DRAW);
		//// position attribute
		//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		//glEnableVertexAttribArray(0);
	}

	// 0 = physical env walls
	// 1 = physical env obstacles
	// 2 = physical user data
	// 3 = optimal node
	// 4 = virt. target node
	unsigned int VBOs[5], VAOs[5], EBOs[5];
	glGenVertexArrays(5, VAOs);
	glGenBuffers(5, VBOs);
	glGenBuffers(5, EBOs);

	// === physical env wall data ===
	glBindVertexArray(VAOs[0]);
	glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);
	glBufferData(GL_ARRAY_BUFFER, config::phys_env1.wall_vertex_buffer_size(), &config::phys_env1.gl_wall_verts[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOs[0]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, config::phys_env1.wall_index_buffer_size(), &config::phys_env1.gl_wall_indices[0], GL_STATIC_DRAW);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// === physical env obstacle data ===
	if (config::phys_env1.get_obstacles().size() > 0) {
		glBindVertexArray(VAOs[1]);
		glBindBuffer(GL_ARRAY_BUFFER, VBOs[1]);
		glBufferData(GL_ARRAY_BUFFER, config::phys_env1.obstacle_vertex_buffer_size(), &config::phys_env1.gl_obstacle_verts[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOs[1]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, config::phys_env1.obstacle_index_buffer_size(), &config::phys_env1.gl_obstacle_indices[0], GL_STATIC_DRAW);
		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
	}

	// === physical user data ===
	int all_users_vertex_buffer_size = sim.users[0]->vertex_buffer_size() * sim.users.size();
	int all_user_index_buffer_size = sim.users[0]->index_buffer_size() * sim.users.size();
	std::vector<float> all_user_vertices;
	std::vector<int> all_user_indices;
	int user_index_offset = 0;
	for (int i = 0; i < sim.users.size(); i++) {
		std::vector<vec2f> v = sim.users[i]->get_draw_vertices(0.0f);
		for (vec2f p : v) {
			all_user_vertices.push_back(p.x);
			all_user_vertices.push_back(p.y);
			all_user_vertices.push_back(0.0f);
		}
		for (int index : sim.users[i]->gl_indices) {
			all_user_indices.push_back(index + (i * sim.users[i]->num_vertices));
		}
		user_index_offset += sim.users[i]->gl_indices.size();
	}
	glBindVertexArray(VAOs[2]);
	glBindBuffer(GL_ARRAY_BUFFER, VBOs[2]);
	glBufferData(GL_ARRAY_BUFFER, all_users_vertex_buffer_size, &all_user_vertices[0], GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOs[2]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, all_user_index_buffer_size, &all_user_indices[0], GL_DYNAMIC_DRAW);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// === optimal node data ===
	std::vector<float> node_verts = std::vector<float>{
		-0.25f, 0.25f, 0.0f,
		-0.25f, -0.25f, 0.0f,
		0.25f, -0.25f, 0.0f,
		0.25f, 0.25f, 0.0f
	};
	std::vector<int> node_indices = std::vector<int>{
		0, 1,
		1, 2,
		2, 3,
		3, 0
	};
	glBindVertexArray(VAOs[3]);
	glBindBuffer(GL_ARRAY_BUFFER, VBOs[3]);
	glBufferData(GL_ARRAY_BUFFER, 12.0f * sizeof(float), &node_verts[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOs[3]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 12.0f * sizeof(int), &node_indices[0], GL_STATIC_DRAW);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// === virt. target node data ===
	std::vector<float> virt_node_verts = std::vector<float>{
		-0.25f, 0.25f, 0.0f,
		-0.25f, -0.25f, 0.0f,
		0.25f, -0.25f, 0.0f,
		0.25f, 0.25f, 0.0f
	};
	std::vector<int> virt_node_indices = std::vector<int>{
		0, 1,
		1, 2,
		2, 3,
		3, 0
	};
	glBindVertexArray(VAOs[4]);
	glBindBuffer(GL_ARRAY_BUFFER, VBOs[4]);
	glBufferData(GL_ARRAY_BUFFER, 12.0f * sizeof(float), &virt_node_verts[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBOs[4]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 12.0f * sizeof(int), &virt_node_indices[0], GL_STATIC_DRAW);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// tell opengl for each sampler to which texture unit it belongs to (only has to be done once)
	// -------------------------------------------------------------------------------------------
	ourShader.use();
	//ourShader.setInt("texture1", 0);
	//ourShader.setInt("texture2", 1);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// === IMGUI STUFF ===
	bool slowmo = false;
	int step_advance_counter = 0;
	char frame_limit_buf[2000] = { 0 };

	// simulation loop
	// ---------------
	static float FPS_cap = 1.0f / 100.0f;
	float prev_time = glfwGetTime();
	float accumulator = 0.0f;
	time_t start_time;
	start_time = time(NULL);
	int frame_count = 0;
	while (!glfwWindowShouldClose(window) && !sim.done) {
		frame_count++;
		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		float new_time = glfwGetTime();
		float frame_time = new_time - prev_time;
		float frame_time_ms = frame_time * 1000.0f;
		if (frame_time > 0.25f) frame_time = 0.25f; // TODO: why do I need this??
		prev_time = new_time;
		if (!slowmo) accumulator += frame_time;

		if (slowmo) {
			if (step_advance_counter > 0) {
				sim.step();

				// Update user position data
				all_user_vertices.clear();
				for (user* user : sim.users) {
					std::vector<vec2f> v = user->get_draw_vertices(0.0f);
					for (vec2f p : v) {
						all_user_vertices.push_back(p.x);
						all_user_vertices.push_back(p.y);
						all_user_vertices.push_back(0.0f);
					}
				}
				glBindBuffer(GL_ARRAY_BUFFER, VBOs[2]);
				glBufferSubData(GL_ARRAY_BUFFER, 0, all_users_vertex_buffer_size, &all_user_vertices[0]);

				step_advance_counter--;
			}
		}
		else {
			//accumulator = 0.0f;
			//new_time = glfwGetTime();
			//frame_time = new_time - prev_time;
			//frame_time_ms = frame_time * 1000.0f;
			//if (frame_time > 0.25f) frame_time = 0.25f; // TODO: why do I need this??
			//prev_time = new_time;
			//accumulator += frame_time;

			while (accumulator >= timestep::dt) {
				sim.step();
				accumulator -= timestep::dt;
			}
			float alpha = accumulator / timestep::dt;

			// Update user position data
			all_user_vertices.clear();
			for (user* user : sim.users) {
				std::vector<vec2f> v = user->get_draw_vertices(alpha);
				for (vec2f p : v) {
					all_user_vertices.push_back(p.x);
					all_user_vertices.push_back(p.y);
					all_user_vertices.push_back(0.0f);
				}
			}
			glBindBuffer(GL_ARRAY_BUFFER, VBOs[2]);
			glBufferSubData(GL_ARRAY_BUFFER, 0, all_users_vertex_buffer_size, &all_user_vertices[0]);
		}

		// Update obstacles position data
		if (config::phys_env1.get_obstacles().size() > 0) {
			glBindBuffer(GL_ARRAY_BUFFER, VBOs[1]);
			glBufferSubData(GL_ARRAY_BUFFER, 0, config::phys_env1.obstacle_vertex_buffer_size(), &config::phys_env1.gl_obstacle_verts[0]);
		}

		// input
		// -----
		processInput(window, sim);

		// Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
		{
			ImGui::Begin("Debug menu");
			ImGui::Text("Step: %d", timestep::num_timesteps);
			ImGui::Checkbox("Slowmo", &slowmo);
			if (ImGui::Button("Advance 1 step"))
				step_advance_counter++;
			if (ImGui::Button("Advance 1000 step"))
				step_advance_counter += 1000;
			ImGui::InputText("Advance until frame #", frame_limit_buf, IM_ARRAYSIZE(frame_limit_buf));
			if (ImGui::Button("Advance custom # steps")) {
				if (atoi(frame_limit_buf) > 0)
					step_advance_counter += atoi(frame_limit_buf);
			}
			ImGui::Checkbox("Show gradient", &draw_gradient);

			ImGui::Text("Apply rotation: %d", sim.users[0]->rdw->apply_rota);
			ImGui::Text("Rotation gain: %f", sim.users[0]->rdw->cur_rota_gain);
			ImGui::Text("Apply translation: %d", sim.users[0]->rdw->apply_trans);
			ImGui::Text("Translation gain: %f", sim.users[0]->rdw->cur_trans_gain);
			ImGui::Text("Apply curvature: %d", sim.users[0]->rdw->apply_curve);
			ImGui::Text("Curvature gain: %f", sim.users[0]->rdw->cur_curve_gain);
			ImGui::Text("Curvature direction: %d", sim.users[0]->rdw->curve_dir);
			//node* op_node = ((vis_graph_rdw*)(sim.users[0]->rdw))->optimal_node;
			node* op_node = new node(vec2f(0.0f, 0.0f));
			//op_node->location.x = 0.0f;
			//op_node->location.y = 0.0f;
			if (op_node == NULL) {
				ImGui::Text("Optimal phys node: (%f, %f)", 0.0f, 0.0f);
			}
			else {
				ImGui::Text("Optimal phys node: (%f, %f)", op_node->location.x, op_node->location.y);
			}
			ImGui::Text("Curvature direction: %d", sim.users[0]->rdw->curve_dir);

			ImGui::Text("Nav state: %d", sim.users[0]->state.nav_state);
			ImGui::Text("Reset timer: %d", sim.users[0]->state.rdw->reset_timer);
			ImGui::Text("Post reset timer: %d", sim.users[0]->state.rdw->post_reset_timer);
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", frame_time_ms, 1.0f / frame_time);
			ImGui::End();
		}

		// render
		// ------
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		// bind textures on corresponding texture units
		glActiveTexture(GL_TEXTURE0);
		//glBindTexture(GL_TEXTURE_2D, texture1);
		glActiveTexture(GL_TEXTURE1);
		//glBindTexture(GL_TEXTURE_2D, texture2);

		// activate shader
		ourShader.use();

		// create transformations
		glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
		glm::mat4 view = glm::mat4(1.0f);
		glm::mat4 projection = glm::mat4(1.0f);
		model = glm::rotate(model, 0.0f, glm::vec3(1.0f, 0.0f, 0.0f));
		view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));
		projection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, 0.1f, 10.0f);
		// retrieve the matrix uniform locations
		unsigned int modelLoc = glGetUniformLocation(ourShader.ID, "model");
		unsigned int viewLoc = glGetUniformLocation(ourShader.ID, "view");
		// pass them to the shaders (3 different ways)
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, &view[0][0]);
		// note: currently we set the projection matrix each frame, but since the projection matrix rarely changes it's often best practice to set it outside the main loop only once.
		ourShader.setMat4("projection", projection);

		// Draw calls
		// Walls
		GLint color_location = glGetUniformLocation(ourShader.ID, "wallColor");
		float color[3] = { 0.0f, 0.0f, 0.0f };
		glUniform3fv(color_location, 1, color);
		glLineWidth(1.0f);
		glBindVertexArray(VAOs[0]);
		glDrawElements(GL_LINE_LOOP, config::phys_env1.gl_wall_indices.size(), GL_UNSIGNED_INT, 0);

		// gradient
		if (draw_gradient && config::DEBUG) {
			int gradient_offset = 0;
			for (int i = 0; i < index_data.size() / 2; i++) {
				glBindVertexArray(vao);
				glDrawElements(GL_LINE_LOOP, 2, GL_UNSIGNED_INT, (void*)gradient_offset);
				gradient_offset += 2 * sizeof(unsigned int);
			}
		}

		// Obstacles
		color_location = glGetUniformLocation(ourShader.ID, "wallColor");
		float color2[3] = { 0.0f, 0.0f, 0.0f };
		glUniform3fv(color_location, 1, color2);
		if (config::phys_env1.get_obstacles().size() > 0) {
			int obstacle_index_offset = 0;
			for (int i = 0; i < config::phys_env1.get_obstacles().size(); i++) {
				obstacle* obs = config::phys_env1.get_obstacles()[i];

				glBindVertexArray(VAOs[1]);
				glDrawElements(GL_LINE_LOOP, obs->gl_indices.size(), GL_UNSIGNED_INT, (void*)obstacle_index_offset);

				obstacle_index_offset += obs->index_buffer_size();
			}
		}

		// Users
		color_location = glGetUniformLocation(ourShader.ID, "wallColor");
		float color3[3] = { 0.0f, 0.0f, 0.0f };
		glUniform3fv(color_location, 1, color3);
		user_index_offset = 0;
		for (user* user : sim.users) {
			int x = user->gl_indices.size();

			glBindVertexArray(VAOs[2]);
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)user_index_offset);

			user_index_offset += user->index_buffer_size();
		}

		// Optimal node
		color_location = glGetUniformLocation(ourShader.ID, "wallColor");
		float color4[3] = { 1.0f, 0.0f, 0.0f };
		glUniform3fv(color_location, 1, color4);
		//node* op_node = ((vis_graph_rdw*)(sim.users[0]->rdw))->optimal_node;
		node* op_node = new node(vec2f(0.0f, 0.0f));
		node_verts = std::vector<float>{
			-0.25f, 0.25f, 0.0f,
			-0.25f, -0.25f, 0.0f,
			0.25f, -0.25f, 0.0f,
			0.25f, 0.25f, 0.0f
		};
		if (op_node != NULL) {
			node_verts[0] += op_node->location.x;
			node_verts[3] += op_node->location.x;
			node_verts[6] += op_node->location.x;
			node_verts[9] += op_node->location.x;

			node_verts[1] += op_node->location.y;
			node_verts[4] += op_node->location.y;
			node_verts[7] += op_node->location.y;
			node_verts[10] += op_node->location.y;
		}
		glBindBuffer(GL_ARRAY_BUFFER, VBOs[3]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, 12.0f * sizeof(float), &node_verts[0]);
		glBindVertexArray(VAOs[3]);
		glDrawElements(GL_LINE_LOOP, node_indices.size(), GL_UNSIGNED_INT, (void*)0);

		// Virtual target node
		color_location = glGetUniformLocation(ourShader.ID, "wallColor");
		float color5[3] = { 0.0f, 1.0f, 0.0f };
		glUniform3fv(color_location, 1, color5);
		node* virt_node = new node(vec2f(0.0f, 0.0f));
		virt_node_verts = std::vector<float>{
			-0.25f, 0.25f, 0.0f,
			-0.25f, -0.25f, 0.0f,
			0.25f, -0.25f, 0.0f,
			0.25f, 0.25f, 0.0f
		};
		if (virt_node != NULL) {
			virt_node_verts[0] += 0.0f;
			virt_node_verts[3] += 0.0f;
			virt_node_verts[6] += 0.0f;
			virt_node_verts[9] += 0.0f;

			virt_node_verts[1] += 0.0f;
			virt_node_verts[4] += 0.0f;
			virt_node_verts[7] += 0.0f;
			virt_node_verts[10] += 0.0f;
		}
		glBindBuffer(GL_ARRAY_BUFFER, VBOs[4]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, 12.0f * sizeof(float), &virt_node_verts[0]);
		glBindVertexArray(VAOs[4]);
		glDrawElements(GL_LINE_LOOP, virt_node_indices.size(), GL_UNSIGNED_INT, (void*)0);

		// ImGui rendering
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	time_t end_time;
	end_time = time(NULL);
	sim.write(frame_count, start_time, end_time);

	// optional: de-allocate all resources once they've outlived their purpose:
	// ------------------------------------------------------------------------
	glDeleteVertexArrays(3, VAOs);
	glDeleteBuffers(3, VBOs);
	glDeleteBuffers(3, EBOs);

	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window, simulation sim)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		sim.quit_early();
		glfwSetWindowShouldClose(window, true);
	}
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

void signal_callback_handler(int signum) {
	cout << "Caught signal " << signum << endl;
	EXIT_EARLY = 1;
}

int run_no_graphics() {
	// Register signal and signal handler
	signal(SIGINT, signal_callback_handler);

	simulation sim = simulation(config::phys_envs, &config::virt_env, config::users);

	time_t start_time;
	start_time = time(NULL);
	int frame_count = 0;

	while (!sim.done || EXIT_EARLY) {
		sim.step();
		frame_count++;

		if (EXIT_EARLY) {
			sim.quit_early();
			break;
		}
	}
	time_t end_time;
	end_time = time(NULL);
	sim.write(frame_count, start_time, end_time);

	std::cout << "Num frames: " << frame_count << std::endl;
	std::cout << "Elapsed real time: " << end_time - start_time << std::endl;
	std::cout << "FPS: " << frame_count / (end_time - start_time) << std::endl;

	return 0;
}

void experiment() {
	if (config::GRAPHICS) run_graphics();
	else run_no_graphics();
}

int main(){
	//test();

	experiment();

	/*float total = config::ARC_V2.constructor_time + config::ARC_V2.update_time;
	std::cout << "constructor time: " << config::ARC_V2.constructor_time << std::endl;
	std::cout << "constructor percent: " << (config::ARC_V2.constructor_time / total) * 100.0f << std::endl;
	std::cout << "get_vis_poly time: " << config::ARC_V2.get_vis_poly_time << std::endl;
	std::cout << "get_vis_poly percent: " << (config::ARC_V2.get_vis_poly_time / total) * 100.0f << std::endl;
	std::cout << "update time: " << config::ARC_V2.update_time << std::endl;
	std::cout << "update percent: " << (config::ARC_V2.update_time / total) * 100.0f << std::endl;
	std::cout << "update_visibility_polygons time: " << config::ARC_V2.update_visibility_polygons_time << std::endl;
	std::cout << "update_visibility_polygons percent: " << (config::ARC_V2.update_visibility_polygons_time / total) * 100.0f << std::endl;
	std::cout << "update time: " << config::ARC_V2.update_time << std::endl;
	std::cout << "update percent: " << (config::ARC_V2.update_time / total) * 100.0f << std::endl;
	std::cout << "get_loss_gradient_direction time: " << config::ARC_V2.get_loss_gradient_direction_time << std::endl;
	std::cout << "get_loss_gradient_direction percent: " << (config::ARC_V2.get_loss_gradient_direction_time / total) * 100.0f << std::endl;
	std::cout << "get_simple_visibility_polygons time: " << config::ARC_V2.get_simple_visibility_polygons_time << std::endl;
	std::cout << "get_simple_visibility_polygons percent: " << (config::ARC_V2.get_simple_visibility_polygons_time / total) * 100.0f << std::endl;
	std::cout << "update_loss time: " << config::ARC_V2.update_loss_time << std::endl;
	std::cout << "update_loss percent: " << (config::ARC_V2.update_loss_time / total) * 100.0f << std::endl;
	std::cout << "get_ray_weight time: " << config::ARC_V2.get_ray_weight_time << std::endl;
	std::cout << "get_ray_weight percent: " << (config::ARC_V2.get_ray_weight_time / total) * 100.0f << std::endl;
	std::cout << "get_proximity_in_visibility_polygon time: " << config::ARC_V2.get_proximity_in_visibility_polygon_time << std::endl;
	std::cout << "get_proximity_in_visibility_polygon percent: " << (config::ARC_V2.get_proximity_in_visibility_polygon_time / total) * 100.0f << std::endl;
	std::cout << "area_of_polygon_with_holes time: " << config::ARC_V2.area_of_polygon_with_holes_time << std::endl;
	std::cout << "area_of_polygon_with_holes percent: " << (config::ARC_V2.area_of_polygon_with_holes_time / total) * 100.0f << std::endl;
	std::cout << "compute_rotation_loss time: " << config::ARC_V2.compute_rotation_loss_time << std::endl;
	std::cout << "compute_rotation_loss percent: " << (config::ARC_V2.compute_rotation_loss_time / total) * 100.0f << std::endl;
	std::cout << "set_gains time: " << config::ARC_V2.set_gains_time << std::endl;
	std::cout << "set_gains percent: " << (config::ARC_V2.set_gains_time / total) * 100.0f << std::endl;
	std::cout << "reset time: " << config::ARC_V2.reset_time << std::endl;
	std::cout << "reset percent: " << (config::ARC_V2.reset_time / total) * 100.0f << std::endl;
	std::cout << "reset_to_distance_alignment time: " << config::ARC_V2.reset_to_distance_alignment_time << std::endl;
	std::cout << "reset_to_distance_alignment percent: " << (config::ARC_V2.reset_to_distance_alignment_time / total) * 100.0f << std::endl;
	std::cout << "get_current_visibility_polygon time: " << config::ARC_V2.get_current_visibility_polygon_time << std::endl;
	std::cout << "get_current_visibility_polygon percent: " << (config::ARC_V2.get_current_visibility_polygon_time / total) * 100.0f << std::endl;
	std::cout << "distance_to_closest_feature time: " << config::ARC_V2.distance_to_closest_feature_time << std::endl;
	std::cout << "distance_to_closest_feature percent: " << (config::ARC_V2.distance_to_closest_feature_time / total) * 100.0f << std::endl;
	std::cout << "reset_to_gradient time: " << config::ARC_V2.reset_to_gradient_time << std::endl;
	std::cout << "reset_to_gradient percent: " << (config::ARC_V2.reset_to_gradient_time / total) * 100.0f << std::endl;
	std::cout << "compute_gradient time: " << config::ARC_V2.compute_gradient_time << std::endl;
	std::cout << "compute_gradient percent: " << (config::ARC_V2.compute_gradient_time / total) * 100.0f << std::endl;
	std::cout << "compute_repulsive_force time: " << config::ARC_V2.compute_repulsive_force_time << std::endl;
	std::cout << "compute_repulsive_force percent: " << (config::ARC_V2.compute_repulsive_force_time / total) * 100.0f << std::endl;*/

	//system("pause");

	return 0;
}