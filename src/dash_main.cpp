#include "async_stdio.hpp"
#include "coro.hpp"
#include <array>
#include <cstdio>
#include <hardware/adc.h>
#include <hardware/address_mapped.h>
#include <hardware/dma.h>
#include <hardware/platform_defs.h>
#include <hardware/regs/adc.h>
#include <hardware/regs/dma.h>
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/rp/adc.hpp>
#include <libhal-arm-mcu/rp/output_pin.hpp>
#include <libhal-arm-mcu/rp/rp.hpp>
#include <libhal-arm-mcu/rp/serial.hpp>
#include <libhal-arm-mcu/rp/time.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <numbers>
#include <pico/error.h>
#include <pico/stdio.h>
#include <span>

struct AppState
{
  std::array<float, 4> readings{};
  std::array<bool, 4> display{ true, true, true, true };

  co::Coroutine<void> run_app()
  {
    std::array<char, 32> buffer;
    for (;;) {
      auto cmd = co_await async_read_str(buffer);
      if (cmd == "t0")
        display[0] = !display[0];
      else if (cmd == "t1")
        display[1] = !display[1];
      else if (cmd == "t2")
        display[2] = !display[2];
      else if (cmd == "t3")
        display[3] = !display[3];
    }
  }
};

int main()
{
  namespace rp = hal::rp;
  using namespace std::chrono_literals;
  auto out = rp::stdio_serial();
  auto clk = rp::clock();
  auto dwt_clk = hal::cortex_m::dwt_counter(rp::core_clock());

  AppState state;
  auto app = state.run_app();
  int n = 0;
  for (;;) {
    app.poll();
    hal::delay(clk, 10ms);
    n = (n + 1) % 100;
    float offset = std::numbers::pi_v<float> * 2.0f / 3.0f;
    state.readings[0] = sinf(0.001f * clk.uptime());
    state.readings[1] = sinf(0.001f * clk.uptime() + offset);
    state.readings[2] = sinf(0.001f * clk.uptime() - offset);
    state.readings[3] = remainderf(0.001f * clk.uptime(), 1.f);
    if (n == 0) {
      for (size_t i = 0; i < state.readings.size(); ++i)
        if (state.display[i])
          hal::print<30>(out, "%d: %f\t", i, state.readings[i]);
      hal::print(out, "\n");
    }
  }
}
