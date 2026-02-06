#include "gui.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/base.h>
#include <stdexcept>
#include <thread>

int main()
{
  try {
    GUI gui;
    State s;
    s.show_demo = true;
    s.data.resize(200, 0.0f);
    using namespace std::chrono_literals;
    auto later = std::chrono::system_clock().now() + 10s;
    size_t i = 0;
    while (std::chrono::system_clock().now() < later) {
      s.data[0] = std::sin(i * 0.1);
      std::rotate(s.data.begin(), s.data.begin() + 1, s.data.end());
      printf("setting %lu to %f\n", i, s.data[i % s.data.size()]);
      i += 1;
      gui.poll(s);
      std::this_thread::sleep_for(10ms);
    }
  } catch (std::runtime_error const& r) {
    fmt::println("Encountered error: {}", r.what());
  }
}
