#include <hr4c/core/logger.hpp>
#include <hr4c/graphics/gui.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <stdio.h>

#include <imgui.h>
// --- 順序関係あり
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

static void error_callback(int error, const char* description) { fprintf(stderr, "Error %d: %s\n", error, description); }

static GLFWwindow* window;

namespace hr4c {

int init_view() {
  glfwSetErrorCallback(error_callback);
  if(!glfwInit()) return 1;
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window = glfwCreateWindow(1080, 700, "ImGui OpenGL3 example", NULL, NULL);
  glfwMakeContextCurrent(window);

  glfwSwapInterval(0); // disable vsync

  glewInit();

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui::StyleColorsDark();

  // Setup ImGui binding
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init();

  // start docking
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigDockingWithShift = false;
  io.ConfigDockingNoSplit   = false;
  return 1;
}

bool newframe_gui() {
  bool should_close = glfwWindowShouldClose(window);
  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  return !should_close;
}

void update_gui() {
  static bool show_demo_window    = true;
  static bool show_another_window = false;
  static ImVec4 clear_color       = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
  if(show_demo_window) ImGui::ShowDemoWindow(&show_demo_window);

  // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
  {
    static float f     = 0.0f;
    static int counter = 0;

    ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

    ImGui::Text("This is some useful text.");          // Display some text (you can use a format strings too)
    ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bools storing our window open/close state
    ImGui::Checkbox("Another Window", &show_another_window);

    ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
    ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

    if(ImGui::Button("Button")) // Buttons return true when clicked (most widgets return true when edited/activated)
      counter++;
    ImGui::SameLine();
    ImGui::Text("counter = %d", counter);

    ImGuiIO& io = ImGui::GetIO();
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
  }

  // 3. Show another simple window.
  if(show_another_window) {
    ImGui::Begin("Another Window", &show_another_window); // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
    ImGui::Text("Hello from another window!");
    if(ImGui::Button("Close Me")) show_another_window = false;
    ImGui::End();
  }
}

void render_gui() {
  ImGui::Render();
  int display_w, display_h;
  glfwGetFramebufferSize(window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(window);
}

void terminate_gui() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  ImPlot::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

void plot_axis(const std::string& name, const AxisBuffer* data) {
  static float history = 1.0f;
  ImGui::SliderFloat("History", &history, 1, 3, "%.1f s");
  auto s = "plot " + name;
  if(ImPlot::BeginPlot(s.c_str(), ImVec2(-1, 200))) {
    ImPlot::SetupAxes("time", "v");
    ImPlot::SetupAxisLimits(ImAxis_X1, data->t - history, data->t, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -300, 300, ImGuiCond_Once);
    // ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
    ImPlot::PlotLine("pos", &data->sd_pos.Data[0].x, &data->sd_pos.Data[0].y, data->sd_pos.Data.size(), 0, data->sd_pos.Offset, 2 * sizeof(float));
    ImPlot::PlotLine("vel", &data->sd_vel.Data[0].x, &data->sd_vel.Data[0].y, data->sd_vel.Data.size(), 0, data->sd_vel.Offset, 2 * sizeof(float));
    ImPlot::PlotLine("tor", &data->sd_tor.Data[0].x, &data->sd_tor.Data[0].y, data->sd_tor.Data.size(), 0, data->sd_tor.Offset, 2 * sizeof(float));
    ImPlot::EndPlot();
  }
}


} // namespace hr4c
