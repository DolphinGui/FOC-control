#include "libhal-arm-mcu/dwt_counter.hpp"
#include "libhal-arm-mcu/rp/output_pin.hpp"
#include "libhal-arm-mcu/rp/serial.hpp"
#include "libhal-arm-mcu/rp/time.hpp"
#include "resource_list.hpp"
#include <libhal/initializers.hpp>

void initialize_platform(resource_list& p_list)
{
  using namespace hal::literals;
  namespace rp = hal::rp;
  static auto timer = hal::cortex_m::dwt_counter(rp::core_clock());

  p_list.clock = &timer;

  static hal::rp::output_pin led(hal::pin<7>);
  p_list.status_led = &led;

  static rp::stdio_serial serial;
  p_list.console = &serial;
}
