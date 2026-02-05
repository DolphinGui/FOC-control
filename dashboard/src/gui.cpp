#include "gui.hpp"
#include "resource.hpp"
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <glad/gl.h>

#include <GLFW/glfw3.h>

#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

static void error_callback(int error, char const* description)
{
  fprintf(stderr, "Error 0x%x: %s\n", error, description);
}

GUI::GUI()
{
  this->inner = std::make_unique<State>();
}

GUI::~GUI()
{
}

namespace {
struct GLFW
{
  GLFW()
  {
    std::printf("init glfw\n");
    glfwSetErrorCallback(error_callback);
    glfwInit();
  }
  GLFW(GLFW&&) = delete;
  GLFW(GLFW const&) = delete;
  ~GLFW()
  {
    std::printf("terminating glfw\n");
    glfwTerminate();
  }
};
}  // namespace
struct GUI::State
{

  constexpr static auto deleter = [](auto w) noexcept {
    if (w)
      glfwDestroyWindow(w);
    std::printf("terminating window: 0x%lx\n", (uint64_t)w);
  };
  using Window = Resource<GLFWwindow*, decltype(deleter)>;
  GLFW g;
  Window window;

  State()
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

    ImGui_ImplOpenGL3_Init();
  }

  ~State()
  {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
  }
};

void GUI::poll()
{
  glfwPollEvents();
  if (glfwGetWindowAttrib(*this->inner->window, GLFW_ICONIFIED) != 0) {
    ImGui_ImplGlfw_Sleep(10);
  }
}
