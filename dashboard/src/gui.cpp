#include "gui.hpp"
#include "imgui.h"
#include "io.hpp"
#include "resource.hpp"
#include "state.hpp"
#include <asio/any_io_executor.hpp>
#include <asio/steady_timer.hpp>
#include <asio/use_awaitable.hpp>
#include <cstdio>
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

    ImGui::StyleColorsLight();

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);

    ImGui_ImplGlfw_InitForOpenGL(*window, true);

    ImGui_ImplOpenGL3_Init("#version 300 es");
  }

  void mainmenu(State& s);
  void serial_popup(State& s);
  asio::awaitable<void> plot(State&);

  ~Internal()
  {
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
  }

  bool show_demo = true;
  std::vector<SerialInfo> serials;
};

GUI::GUI()
{
  this->inner = std::make_unique<Internal>();
}

GUI::~GUI()
{
}

asio::awaitable<bool> GUI::poll(State& s)
{
  glfwPollEvents();
  if (glfwGetWindowAttrib(*this->inner->window, GLFW_ICONIFIED) != 0) {
    ImGui_ImplGlfw_Sleep(10);
  }

  if (glfwWindowShouldClose(*this->inner->window)) {
    co_return false;
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  if (inner->show_demo)
    ImGui::ShowDemoWindow(&inner->show_demo);

  inner->mainmenu(s);
  co_await inner->plot(s);

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
  co_return true;
}

void GUI::Internal::mainmenu(State& s)
{
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::Button("Connect Device")) {
      ImGui::OpenPopup("serial_select");
      this->serials = list_serial_devices();
    }
    serial_popup(s);
    ImGui::EndMainMenuBar();
  }
}

void GUI::Internal::serial_popup(State& s)
{
  if (!this->serials.empty()) {
    if (ImGui::BeginPopupModal("serial_select", nullptr)) {
      ImGui::Text("Select the BLDC motor from the items");
      int select_index = 0;
      if (ImGui::BeginListBox("Select Serial")) {
        int i = 0;
        for (auto& se : serials) {
          bool const selected = i == select_index;
          if (ImGui::Selectable(se.name.c_str(), selected)) {
            select_index = i;
          }
          if (selected)
            ImGui::SetItemDefaultFocus();
          i += 1;
        }
        ImGui::EndListBox();
      }
      if (ImGui::Button("Select")) {
        s.connect_device(serials[select_index]);
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
  } else {
    if (ImGui::BeginPopup("serial_select")) {
      ImGui::Text("No devices connected");
      ImGui::EndPopup();
    }
  }
}

asio::awaitable<void> GUI::Internal::plot(State& s)
{
  if (ImGui::Begin("Serial Plot")) {
    ImPlot::BeginPlot("A plot");
    ImPlot::SetupAxes("time", "mag");
    ImPlot::SetupAxesLimits(0, 200, -1, 1);
    auto a = s.list_data();
    for (auto&& [name, set] : co_await s.list_data()) {
      ImPlot::PlotLine(name.data(), set->data.data(), set->data.size());
    }
    ImPlot::EndPlot();
  }
  ImGui::End();
}
