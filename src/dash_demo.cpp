#include <array>
#include <cmath>
#include <libhal-arm-mcu/rp/output_pin.hpp>
#include <libhal-arm-mcu/rp/rp.hpp>
#include <libhal-arm-mcu/rp/serial.hpp>
#include <libhal-arm-mcu/rp/time.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <numbers>
#include <span>

#include "base64.hpp"
#include "conststr.hpp"
#include "coro.hpp"

constexpr int led_pin = hal::rp::internal::pin_max == 30 ? 7 : 46;
namespace rp = hal::rp;

std::string_view str_cast(std::span<hal::byte> b)
{
  return std::string_view(reinterpret_cast<char*>(b.data()), b.size_bytes());
}

struct read_until : co::Task<std::span<hal::byte>>
{
  read_until(hal::serial& se, std::span<hal::byte> out, char end)
    : output(out)
    , serial(&se)
    , end(end)
  {
  }
  std::span<hal::byte> output;
  hal::serial* serial;
  size_t bytes = 0;
  char end;

  co::Microseconds poll() override
  {
    auto g = serial->read(output.subspan(bytes));

    for (size_t i = 0; i < g.data.size(); ++i) {
      if (g.data[i] == end) {
        bytes += i;
        return co::Microseconds(0);
      }
    }

    bytes += g.data.size_bytes();
    if (bytes >= output.size_bytes())
      return co::Microseconds(0);

    return co::Microseconds(100000);
  }

  std::span<hal::byte> value() override
  {
    return output.subspan(0, bytes);
  }
};

using namespace util::string_literal;
using namespace std::chrono_literals;
constexpr auto schema = b64::create_schema<b64::parameter<float, "phase A"_c>,
                                           b64::parameter<float, "phase B"_c>,
                                           b64::parameter<float, "phase C"_c>>();

static co::Coroutine<void> blinky(hal::steady_clock& clk)
{
  auto pin = rp::output_pin(hal::pin<led_pin>);
  bool on = false;
  for (;;) {
    auto later = hal::future_deadline(clk, 1s);
    pin.level(on);
    on = !on;
    while (clk.uptime() < later)
      co_yield 1s;
  }
}

static co::Coroutine<void> print_schema(hal::serial& se)
{
  std::array<hal::byte, 32> data = {};
  for (;;) {
    auto line = str_cast(co_await read_until(se, data, '\n'));
    if (line == "getschema") {
      se.write(std::span(reinterpret_cast<uint8_t const*>(schema.first.data()),
                         schema.second));
    }
  }
}

struct Job
{
  co::Coroutine<void> task;
  co::Microseconds wake_time = co::Microseconds{ 0 };
};

int main()
{
  using namespace std::chrono_literals;
  auto out = rp::stdio_serial();
  auto clk = rp::clock();
  auto pin = rp::output_pin(hal::pin<led_pin>);
  float w = 1.5f / clk.frequency();
  float const phase_offset = std::numbers::pi * 2. / 3.;

  auto bl = blinky(clk);
  auto pr = print_schema(out);

  for (;;) {

    bl.poll();
    pr.poll();
    auto angle = clk.uptime() * w;
    auto [data, len] = b64::encode_message(
      sinf(angle), sinf(angle - phase_offset), sinf(angle + phase_offset));
    out.write(std::span(reinterpret_cast<uint8_t*>(data.data()), len));

    rp::sleep(1ms);
  }
}
