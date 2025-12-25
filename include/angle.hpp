#pragma once

#include "utility.hpp"
#include <mp-units/systems/angular/units.h>
#include <mp-units/systems/si/math.h>

namespace foc {

struct v_ab
{
  volts a, b;
};

inline v_ab clarke(volts u, volts v, volts h) noexcept
{
  volts a = sqrtf(2.0f / 3.0f) * (u - 0.5f * v - 0.5f * h);
  volts b = sqrtf(2.0f) / 2.0f * (v - h);
  return { a, b };
}

// algorithm based on
// https://davidmolony.github.io/MESC_Firmware/operation/CONTROL.html#the-sensorless-observer
struct flux_linkage_observer
{
  constexpr flux_linkage_observer(volts vin, motor_characteristics m)
    : v_in(vin)
    , r_m(m.phase_resistance)
    , l_m(m.phase_inductance)
  {
    // https://ww2.mathworks.cn/help/sps/ug/parameterize-a-permanent-magnet-synchronous-motor.html
    // for some reason simpleFOC adds a sqrt(3) coefficient, need to test to see
    // if this is accurate
    using namespace mp_units;
    using namespace mp_units::si::unit_symbols;
    using namespace mp_units::angular::unit_symbols;
    quantity a = m.kv.in(mp_units::angular::radian / mp_units::si::second /
                         mp_units::si::volt) /
                 mp_units::angular::radian;
    flux_linkage = 1 / (a * m.pole_pairs);
  }

  radian estimate_angle(milliseconds dt, triple_hbridge const& motor, ab i)
  {
    using namespace mp_units::si::unit_symbols;
    v_ab v_m = clarke(v_in * motor.u.get_duty_tristate(),
                      v_in * motor.v.get_duty_tristate(),
                      v_in * motor.h.get_duty_tristate());
    volts emf_a = v_m.a - r_m * i.a - l_m * (i.a - prev_a) / dt;
    volts emf_b = v_m.b - r_m * i.b - l_m * (i.b - prev_b) / dt;
    phi_a = saturate(phi_a + emf_a * dt, flux_linkage);
    phi_b = saturate(phi_b + emf_b * dt, flux_linkage);
    prev_a = i.a;
    prev_b = i.b;
    return mp_units::si::atan2(phi_b, phi_a) + (180.0f * deg).in(rad);
  }

private:
  using volt_sec =
    mp_units::quantity<mp_units::si::volt *
                         mp_units::si::milli<mp_units::si::second>,
                       float>;
  amps prev_a{}, prev_b{};
  volt_sec phi_a{}, phi_b{}, flux_linkage;

  volts v_in;
  // both values are phase-to-center, divide by 2 from phase-to-phase values
  ohms r_m;
  henries l_m;
};
}  // namespace foc