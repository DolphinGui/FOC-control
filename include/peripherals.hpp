#pragma once

#include "utility.hpp"

namespace foc {

struct triple_hbridge
{
  // -1.0 is ground, 1.0 is vin, 0.0 is high-Z
  virtual void set_duty(uvh<mp_units::one>) = 0;
  // incapable of high-Z output
  virtual void set_duty_bipolar(uvh<mp_units::one, uint16_t>) = 0;
  virtual uvh<mp_units::one> get_duty() const = 0;

  virtual ~triple_hbridge() = default;
};

struct triple_current_sensor
{
  virtual uvh<mp_units::si::ampere> get_current() const = 0;
  virtual ~triple_current_sensor() = default;
};

struct encoder
{
  virtual radians get_angle() const = 0;
  virtual ~encoder() = default;
};
}  // namespace foc