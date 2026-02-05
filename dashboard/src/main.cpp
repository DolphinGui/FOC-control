#include "gui.hpp"
#include <chrono>
#include <fmt/base.h>
#include <stdexcept>
#include <thread>

int main()
{
  try {
    GUI gui;
    using namespace std::chrono_literals;
    auto later = std::chrono::system_clock().now() + 1s;
    while (std::chrono::system_clock().now() < later) {
      gui.poll();
    }
  } catch (std::runtime_error const& r) {
    fmt::println("Encountered error: {}", r.what());
  }
}
