#include <array>
#include <cstdint>
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/output_pin.hpp>
#include <libhal-arm-mcu/lpc40/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal/initializers.hpp>

#include "resource_list.hpp"

void initialize_platform(resource_list& p_list)
{
  using namespace hal::literals;

  // Set the MCU to the maximum clock speed
  hal::lpc40::maximum(10_MHz);

  static auto counter = hal::cortex_m::dwt_counter(
    hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));
  p_list.clock = &counter;

  static hal::lpc40::output_pin led('C', 13);
  p_list.status_led = &led;

  static std::array<uint8_t, 128> buffer;

  static hal::lpc40::uart uart1(1,
                                buffer,
                                {
                                  .baud_rate = 115200,
                                });
  p_list.console = &uart1;
}
