#pragma once

#include "utility.hpp"

namespace foc {

struct triple_hbridge
{
  virtual void set_duty(uvh<mp_units::one>) = 0;
  virtual void set_duty(uvh<mp_units::one, uint16_t>) = 0;

  virtual ~triple_hbridge() = default;
};

struct triple_current_sensor
{
  virtual uvh<mp_units::si::ampere> get_current() = 0;
  virtual ~triple_current_sensor() = default;
};

struct encoder
{
  virtual radians get_angle() = 0;
  virtual ~encoder() = default;
};
}  // namespace foc