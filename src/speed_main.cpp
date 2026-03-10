#include <hbridge.hpp>

#include <hardware/gpio.h>
#include <hardware/pwm.h>

float const freq = 100;

int main()
{
  namespace rp = hal::rp;
  using namespace std::chrono_literals;
  auto out = rp::stdio_serial();
  auto dwt_clk = hal::cortex_m::dwt_counter(rp::core_clock());
  float w = freq / dwt_clk.frequency();
  triple_hbridge h;
  auto ttt = dwt_clk.uptime();
  hal::u32 count = 0;

  for (;;) {
    hal::u64 time = dwt_clk.uptime();
    float angle = w * dwt_clk.uptime();
    float const twopi = std::numbers::pi * 2.0f / 3.0f;
    float max_duty = 0.5f;
    float a = max_duty * cosf(angle), b = max_duty * cosf(angle - twopi),
          c = max_duty * cosf(angle + twopi);
    if ((time - ttt) / dwt_clk.frequency() > 1) {
      hal::print<50>(out, "%d: [ %.02f %.02f %.02f ]\n", count, a, b, c);
      ttt = time;
      count = 0;
    }

    h.set_duty(a, b, c);
    count += 1;
  }
}