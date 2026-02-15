#include "mock_peripherals.hpp"

void mock_hbridge::set_duty(foc::uvh<mp_units::one> v)
{
  dut = v;
}
void mock_hbridge::set_duty_bipolar(foc::uvh<mp_units::one, uint16_t> v)
{
  using namespace mp_units;
  foc::uvh<one> d{ v.u.in<float>(), v.v.in<float>(), v.h.in<float>() };
  dut =
    (d / (float(std::numeric_limits<uint16_t>::max()) * one)) * (2.0f * one) -
    foc::uvh<one>{ 1.0f * one, 1.0f * one, 1.0f * one };
}

foc::uvh<mp_units::one> mock_hbridge::get_duty() const
{
  return dut;
};
foc::uvh<mp_units::one> dut;

foc::uvh<mp_units::si::ampere> mock_shunt::get_current() const
{
  using namespace mp_units::si::unit_symbols;
  return { 0.5 * A, 0.4 * A, -0.9 * A };
}

foc::radians mock_encoder::get_angle() const
{
  using namespace mp_units::si::unit_symbols;
  return 1.2 * rad;
}
