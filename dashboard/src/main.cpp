#include "gui.hpp"
#include <chrono>
#include <fmt/base.h>
#include <stdexcept>
#include <thread>

int main()
{
  try {
    GUI gui;
    State s;
    s.show_demo = true;
    using namespace std::chrono_literals;
    auto later = std::chrono::system_clock().now() + 10s;
    while (std::chrono::system_clock().now() < later) {
      gui.poll(s);
    }
  } catch (std::runtime_error const& r) {
    fmt::println("Encountered error: {}", r.what());
  }
}
