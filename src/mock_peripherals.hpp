#pragma once

#include "peripherals.hpp"
#include "vectors.hpp"
#include <limits>

struct mock_hbridge final : foc::triple_hbridge
{

  void set_duty(foc::uvh<mp_units::one> v) override;
  void set_duty_bipolar(foc::uvh<mp_units::one, uint16_t> v) override;

  foc::uvh<mp_units::one> get_duty() const override;
  ~mock_hbridge() override = default;
  foc::uvh<mp_units::one> dut;
};

struct mock_shunt final : foc::triple_current_sensor
{

  foc::uvh<mp_units::si::ampere> get_current() const override;
  ~mock_shunt() override = default;
};

struct mock_encoder final : foc::encoder
{
  foc::radians get_angle() const override;
  ~mock_encoder() override = default;
};