#include "gui.hpp"
#include "resource.hpp"
#include <memory>
#include <stdexcept>

#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "implot/implot.h"

#include <GLFW/glfw3.h>

static void error_callback(int error, char const* description)
{
  fprintf(stderr, "Error 0x%x: %s\n", error, description);
}

GUI::GUI()
{
  this->inner = std::make_unique<Internal>();
}

GUI::~GUI()
{
}

namespace {
struct GLFW
{
  GLFW()
  {
    glfwSetErrorCallback(error_callback);
    glfwInit();
  }
  GLFW(GLFW&&) = delete;
  GLFW(GLFW const&) = delete;
  ~GLFW()
  {
    glfwTerminate();
  }
};
}  // namespace
struct GUI::Internal
{

  constexpr static auto deleter = [](auto w) noexcept {
    if (w)
      glfwDestroyWindow(w);
  };
  using Window = Resource<GLFWwindow*, decltype(deleter)>;
  GLFW g;
  Window window;

  Internal()
  {
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);

    float main_scale =
      ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
    GLFWvidmode const* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int width = mode->width * main_scale * 0.5;
    int height = mode->height * main_scale * 0.5;

    auto* w =
      glfwCreateWindow(width, height, "Motor Dashboard", nullptr, nullptr);
    if (!w)
      throw std::runtime_error("Could not create GLFW window");
    window = Window{ w, deleter };
    glfwMakeContextCurrent(*window);
    glfwSwapInterval(1);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls

    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);

    ImGui_ImplGlfw_InitForOpenGL(*window, true);

    ImGui_ImplOpenGL3_Init("#version 300 es");
  }

  ~Internal()
  {
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
  }
};

void GUI::poll(::State& s)
{
  glfwPollEvents();
  if (glfwGetWindowAttrib(*this->inner->window, GLFW_ICONIFIED) != 0) {
    ImGui_ImplGlfw_Sleep(10);
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  if (s.show_demo)
    ImGui::ShowDemoWindow(&s.show_demo);

  // plot stuff
  ImPlot::BeginPlot("A plot");
  // void PlotLine(const char* label_id, const T* values, int count, double
  // xscale, double x0, ImPlotLineFlags flags, int offset, int stride) {
  ImPlot::SetupAxes("time", "mag");
  ImPlot::SetupAxesLimits(0, 200, -1, 1);
  ImPlot::PlotLine("Angle", s.data.data(), s.data.size());
  ImPlot::EndPlot();

  // ImPlot::ShowDemoWindow();

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  ImGui::Render();
  int display_w, display_h;
  auto window = *this->inner->window;
  glfwGetFramebufferSize(window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(clear_color.x * clear_color.w,
               clear_color.y * clear_color.w,
               clear_color.z * clear_color.w,
               clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(window);
}
