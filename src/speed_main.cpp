#include <array>
#include <cstdint>
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/rp/i2c.hpp>
#include <libhal-arm-mcu/rp/output_pin.hpp>
#include <libhal-arm-mcu/rp/pwm.hpp>
#include <libhal-arm-mcu/rp/serial.hpp>
#include <libhal-arm-mcu/rp/time.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/initializers.hpp>
#include <libhal/units.hpp>
#include <span>

#include <hardware/gpio.h>
#include <hardware/pwm.h>

float const freq = 100;

/*
A H/L: SPI1-CIPO / PWM0    > G44 / G31
B H/L: SPI1-COPI / UART-RX > G43 / G13
C H/L: A0        / UART-TX > G40 / G12
*/
// A0, COPI, CIPO1

struct triple_hbridge
{
  triple_hbridge()
    : s10(hal::channel<10>)
    , s7(hal::channel<7>)
    , s9(hal::channel<9>)
    , s6(hal::channel<6>)
    , s8(hal::channel<8>)
    , ah(s10.get_pin(hal::pin<44>, { .autostart = false }))
    , al(s7.get_pin(hal::pin<31>, { .autostart = false }))
    , bh(s9.get_pin(hal::pin<43>, { .autostart = false }))
    , bl(s6.get_pin(hal::pin<13>, { .autostart = false }))
    , ch(s8.get_pin(hal::pin<40>, { .autostart = false }))
    , cl(s6.get_pin(hal::pin<12>, { .autostart = false }))
  {
    s10.frequency(10'000);
    s7.frequency(10'000);
    s9.frequency(10'000);
    s6.frequency(10'000);
    s8.frequency(10'000);
    hal::rp::v5::enable_all_pwm(true);
  }

  void set_duty(float a, float b, float c)
  {
    hal::u16 const max = 0xffff;
    if (a > 0.f) {
      ah.duty_cycle(max * a);
      al.duty_cycle(fabsf(max * a));
    } else {
      ah.duty_cycle(0);
      al.duty_cycle(fabsf(max * a));
    }
    if (b > 0.f) {
      bh.duty_cycle(max * b);
      bl.duty_cycle(0);
    } else {
      bh.duty_cycle(0);
      bl.duty_cycle(fabsf(max * b));
    }
    if (c > 0.f) {
      ch.duty_cycle(max * c);
      cl.duty_cycle(0);
    } else {
      ch.duty_cycle(0);
      cl.duty_cycle(fabsf(max * c));
    }
  }

private:
  hal::rp::v5::pwm_slice<10> s10;
  hal::rp::v5::pwm_slice<7> s7;
  hal::rp::v5::pwm_slice<9> s9;
  hal::rp::v5::pwm_slice<6> s6;
  hal::rp::v5::pwm_slice<8> s8;
  hal::rp::v5::pwm_pin ah, al, bh, bl, ch, cl;
};

int main()
{
  namespace rp = hal::rp;
  using namespace std::chrono_literals;
  auto out = rp::stdio_serial();
  auto dwt_clk = hal::cortex_m::dwt_counter(rp::core_clock());
  float w = freq / dwt_clk.frequency();
  triple_hbridge h;
  auto ttt = dwt_clk.uptime();

  auto chan = rp::v5::pwm_slice(hal::channel<7>);
  chan.frequency(50'000);
  auto pin = chan.get_pin(hal::pin<31>, { .autostart = true });
  pin.duty_cycle(0xffff * 0.4);
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