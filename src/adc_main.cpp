#include <array>
#include <hardware/adc.h>
#include <hardware/address_mapped.h>
#include <hardware/dma.h>
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

namespace rp = hal::rp;

int main()
{
  using namespace std::chrono_literals;
  auto out = rp::stdio_serial();
  auto clk = rp::clock();
  auto dwt_clk = hal::cortex_m::dwt_counter(SYS_CLK_HZ);
  auto led = rp::output_pin(hal::pin<7>);
  bool on = false;
  auto adc = rp::nonstandard::adc16_pack(
    hal::pin<26>, hal::pin<27>, hal::pin<28>, hal::pin<29>);
  std::array<hal::u16, 4> readings;
  auto read_session = adc.async();

  for (;;) {
    auto a = read_session.read(readings);
    size_t times_polled = 0;
    auto start = dwt_clk.uptime();
    while (a.poll() != 0us) {
      times_polled += 1;
    }
    auto end = dwt_clk.uptime();
    std::array<float, 4> results;
    for (int i = 0; i < 4; ++i) {
      results[i] = readings[i] * 3.3f / (1 << 12);
    }

    led.level(on);
    on = !on;
    hal::print<100>(out,
                    "Voltage level: [%f %f %f %f]\n",
                    results[0],
                    results[1],
                    results[2],
                    results[3]);
    hal::print<50>(out,
                   "Times polled: %d, time waited: %5f\n",
                   times_polled,
                   (end - start) / float(SYS_CLK_MHZ));
    hal::delay(clk, 1s);
  }
}