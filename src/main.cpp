// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include <resource_list.hpp>

#include "angle.hpp"
#include "example_motor.hpp"
#include "foc.hpp"
#include "mock_peripherals.hpp"
#include "utility.hpp"
#include <mp-units/compat_macros.h>

resource_list resources{};

void application();

int main()
{
  initialize_platform(resources);

  try {
    application();
  } catch (std::bad_optional_access const& e) {
    if (resources.console) {
      hal::print(*resources.console.value(),
                 "A resource required by the application was not available!\n"
                 "Calling terminate!\n");
    }
  }
  std::terminate();
}

void application()
{
  using namespace mp_units::angular::unit_symbols;
  using namespace mp_units::si;
  using namespace mp_units;
  using namespace std::chrono_literals;

  auto& led = *resources.status_led.value();
  auto& clock = *resources.clock.value();
  // auto& console = *resources.console.value();

  for(;;){
    led.level(true);
    hal::delay(clock, 1s);
    led.level(false);
    // hal::print(console, "Hello World\n");
    hal::delay(clock, 1s);
  }

  // hal::print(console, "Starting Application!\n");

  // hal::u64 prev_time = clock.uptime();

  // mock_hbridge h;
  // mock_shunt s;
  // mock_encoder e;
  // sensorless_motor m = sensorless_motor::create(clock, &h, &s, &e);

  // size_t i = 0;

  // for (;;) {
  //   hal::u64 now = clock.uptime();
  //   hal::u64 delta_time = now - prev_time;
  //   prev_time = now;
  //   quantity<milli<second>, float> dt = delta_time / clock.frequency() * second;
  //   m.loop(dt);
  //   ++i;
  //   if (i >= 1024) {
  //     i = 0;
  //     hal::print<128>(console,
  //                     "Loop time in milliseconds: %f",
  //                     double(dt.numerical_value_in(milli<second>)));
  //   }
  // }
}
