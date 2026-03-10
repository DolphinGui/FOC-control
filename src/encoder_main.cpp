#include "base64.hpp"
#include <array>
#include <cstdint>
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/rp/i2c.hpp>
#include <libhal-arm-mcu/rp/output_pin.hpp>
#include <libhal-arm-mcu/rp/rp.hpp>
#include <libhal-arm-mcu/rp/serial.hpp>
#include <libhal-arm-mcu/rp/time.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <span>

uint8_t const addr = 0x40;

float get_angle(hal::i2c& i2c)
{
  std::array<uint8_t const, 1> reg{ 0x0e };
  std::array<uint8_t, 2> data = { 0xff, 0xff };
  i2c.transaction(addr, reg, data);
  uint16_t angle = (data[0] << 8) | data[1];
  return angle / float(0xfff) * 360;
}

uint8_t get_gain(hal::i2c& i2c)
{
  std::array<uint8_t const, 1> reg{ 0x1a };
  std::array<uint8_t, 1> data = { 0xff };
  i2c.transaction(addr, reg, data);
  return data[0];
}

int main()
{
  namespace rp = hal::rp;
  using namespace std::chrono_literals;
  auto out = rp::stdio_serial();
  auto clk = rp::clock();
  auto dwt_clk = hal::cortex_m::dwt_counter(rp::core_clock());
  auto led = rp::output_pin(hal::pin<7>);
  auto i2c = rp::i2c(hal::pin<16>, hal::pin<17>, hal::bus<0>);
  i2c.configure({ .clock_rate = 1'000'000 });
  bool led_on = false;
  for (;;) {
    led.level(led_on);
    led_on = !led_on;
    try {
      float angle = get_angle(i2c);
      uint8_t gain = get_gain(i2c);
      auto [data, len] = b64::encode_message(angle, gain);
      out.write(std::span(reinterpret_cast<uint8_t*>(data.data()), len));
    } catch (hal::exception const& e) {
      std::array<uint8_t, 4> err{ 'E', 'R', 'R', '\n' };
      out.write(err);
    }
    hal::delay(dwt_clk, 1s);
  }
}