#pragma once

#include "peripherals.hpp"
#include "vectors.hpp"
#include <limits>

struct mock_hbridge final : foc::triple_hbridge
{

  virtual void set_duty(foc::uvh<mp_units::one> v)
  {
    dut = v;
  }
  virtual void set_duty_bipolar(foc::uvh<mp_units::one, uint16_t> v)
  {
    using namespace mp_units;
    foc::uvh<one> d{ v.u.in<float>(), v.v.in<float>(), v.h.in<float>() };
    dut =
      (d / (float(std::numeric_limits<uint16_t>::max()) * one)) * (2.0f * one) -
      foc::uvh<one>{ 1.0f * one, 1.0f * one, 1.0f * one };
  }

  virtual foc::uvh<mp_units::one> get_duty() const
  {
    return dut;
  };
  foc::uvh<mp_units::one> dut;
};

struct mock_shunt final : foc::triple_current_sensor
{

  virtual foc::uvh<mp_units::si::ampere> get_current() const
  {
    using namespace mp_units::si::unit_symbols;
    return { 0.5 * A, 0.4 * A, -0.9 * A };
  }
};

struct mock_encoder final : foc::encoder
{
  virtual foc::radians get_angle() const
  {
    using namespace mp_units::si::unit_symbols;
    return 1.2 * rad;
  }
};