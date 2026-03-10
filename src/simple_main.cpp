#include <hbridge.hpp>

#include "base64.hpp"
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <libhal-arm-mcu/rp/time.hpp>
#include <numbers>

static float const freq = 100;

static float get_angle(hal::i2c& i2c)
{
  uint8_t const addr = 0x40;
  std::array<uint8_t const, 1> reg{ 0x0e };
  std::array<uint8_t, 2> data = { 0xff, 0xff };
  i2c.transaction(addr, reg, data);
  uint16_t angle = (data[0] << 8) | data[1];
  return (angle / float(0xfff) * 2.0f * std::numbers::pi_v<float>);
}

int main()
{
  namespace rp = hal::rp;
  using namespace std::chrono_literals;
  auto out = rp::stdio_serial();
  auto dwt_clk = hal::cortex_m::dwt_counter(rp::core_clock());
  triple_hbridge h;
  auto ttt = dwt_clk.uptime();
  auto i2c = rp::i2c(
    hal::pin<16>, hal::pin<17>, hal::bus<0>, { .clock_rate = 1'000'000 });

  hal::u32 count = 0;

  h.set_duty(1.0, -1.0, -1.0);
  hal::rp::sleep(std::chrono::duration<hal::u32, std::micro>(50));
  h.set_duty(0.0, 0.0, 0.0);
  float const zero_angle = get_angle(i2c);

  for (;;) {
    hal::u64 time = dwt_clk.uptime();
    float mechanical_angle = (get_angle(i2c) - zero_angle);
    float electrical_angle = mechanical_angle * 14.0f;
    float quadrature = electrical_angle + std::numbers::pi_v<float> / 2.f;
    float const offset = std::numbers::pi_v<float> * 2.0f / 3.0f;
    float max_duty = 0.5f;
    float a = max_duty * cosf(quadrature),
          b = max_duty * cosf(quadrature - offset),
          c = max_duty * cosf(quadrature + offset);
    if ((time - ttt) / dwt_clk.frequency() > 0.1f) {
      auto [data, len] =
        b64::encode_message(count, mechanical_angle, electrical_angle);
      out.write(std::span(reinterpret_cast<uint8_t*>(data.data()), len));
      ttt = time;
      count = 0;
    }

    h.set_duty(a, b, c);
    count += 1;
  }
}
