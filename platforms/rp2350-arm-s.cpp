#include "libhal-arm-mcu/rp/output_pin.hpp"
#include "libhal-arm-mcu/rp/serial.hpp"
#include "libhal-arm-mcu/rp/time.hpp"
#include "resource_list.hpp"
#include <libhal/initializers.hpp>

void initialize_platform(resource_list& p_list)
{
  using namespace hal::literals;
  namespace rp = hal::rp;
  static rp::clock timer;

  p_list.clock = &timer;

  static hal::rp::output_pin led(hal::pin<7>);
  p_list.status_led = &led;

  static rp::stdio_serial serial;
  p_list.console = &serial;
}
