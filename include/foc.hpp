#pragma once

#include <limits>
#include <mp-units/framework.h>
#include <mp-units/framework/quantity_cast.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/math.h>
#include <mp-units/systems/si/unit_symbols.h>
#include <mp-units/systems/si/units.h>

#include <mp-units/math.h>

#include "metaprogramming.hpp"
#include "utility.hpp"
/*
General FOC rundown:

Field Oriented Control uses software to detect the position of the rotor,
and then orients the stator magnetic field to be 90 degrees of the rotor
to minimize wasted torque.

*/
namespace foc {
struct uvh_duty
{
  uint16_t u, v, h;
};

struct dq0
{
  amps d, q;
};

inline ab clarke(uvh i) noexcept
{
  amps a = sqrtf(2.0f / 3.0f) * (i.u - 0.5f * i.v - 0.5f * i.h);
  amps b = sqrtf(2.0f) / 2.0f * (i.v - i.h);
  return { a, b };
}

inline dq0 park(ab i, radian theta)
{
  auto c = mp_units::si::cos(theta);
  auto s = mp_units::si::sin(theta);
  return { c * i.a + s * i.b, -s * i.a + c * i.b };
}

// todo fix to reduce trig function usage if this proves to be an issue
inline uvh inverse_clark_park(dq0 i, radian theta) noexcept
{
  using namespace mp_units;
  using namespace mp_units::si::unit_symbols;
  radian phase_offset = 2.0f / 3.0f * M_PI * si::radian;

  amps a = si::cos(theta) * i.d - si::sin(theta) * i.q;
  amps b =
    si::cos(theta - phase_offset) * i.q - si::sin(theta - phase_offset) * i.d;
  amps c =
    si::cos(theta + phase_offset) * i.q - si::sin(theta + phase_offset) * i.d;
  return { a, b, c };
}

// See table 2 of https://ww1.microchip.com/downloads/en/appnotes/00955a.pdf
// todo stare at assembly and maybe save 1 trig operation
inline uvh_duty space_vector(dq0 i, radian theta, amps max_current)
{
  using namespace mp_units::si::unit_symbols;
  using namespace mp_units;
  using duty = quantity<one, float>;

  radian ang = si::atan2(i.q, i.d) + theta;
  float k = 2.0f / sqrtf(3.0f);
  duty m = hypot(i.q, i.d) / max_current * k;
  if (m.numerical_value_in(one) > 1.0f) {
    m = 1.0f * one;
  }

  quantity<one, int> sector;
  quantity psi = remquo(ang, (60.0f * deg).in(rad), &sector);

  duty a_duty = m * si::sin(60 * deg - psi);
  duty b_duty = m * si::sin(psi);

  constexpr static uint16_t ts = std::numeric_limits<uint16_t>::max();
  uint16_t ta = a_duty.numerical_value_in(one) * ts;
  uint16_t tb = b_duty.numerical_value_in(one) * ts;
  uint16_t t02 = (ts - ta - tb) / 2;

  switch (sector.numerical_value_in(one)) {
    case 0:
      return uvh_duty{ t02,
                       static_cast<uint16_t>(t02 + ta),
                       static_cast<uint16_t>(ts - t02) };
    case 1:
      return uvh_duty{ static_cast<uint16_t>(t02 + tb),
                       t02,
                       static_cast<uint16_t>(ts - t02) };
    case 2:
      return uvh_duty{ static_cast<uint16_t>(ts - t02),
                       t02,
                       static_cast<uint16_t>(t02 + ta) };
    case 3:
      return uvh_duty{ static_cast<uint16_t>(ts - t02),
                       static_cast<uint16_t>(t02 + tb),
                       t02 };
    case 4:
      return uvh_duty{ static_cast<uint16_t>(t02 + ta),
                       static_cast<uint16_t>(ts - t02),
                       t02 };
    case 5:
      return uvh_duty{ t02,
                       static_cast<uint16_t>(ts - t02),
                       static_cast<uint16_t>(t02 + tb) };
    default:
      std::terminate();
  }
}

struct foc
{
  // See
  // https://davidmolony.github.io/MESC_Firmware/operation/CONTROL.html#the-foc-pi
  // for gain calculations
  foc(motor_characteristics m,
      rpm max_speed,
      volts vin,
      float max_duty,
      triple_hbridge* bridge)
    : motor(bridge)
    , i_max(vin / (m.phase_resistance * 2))
    , max_duty(max_duty)
  {
    using namespace mp_units::si::unit_symbols;
    auto p = (m.phase_inductance * max_speed)
               .numerical_value_in(mp_units::angular::radian * V / A);
    auto i = (m.phase_resistance / m.phase_inductance).numerical_value_in(Hz);
    // todo add saturation current
    d_pid = zero_pi_controller<mp_units::si::ampere>(p, i, {}, 10.f);
    q_pid = pi_controller<mp_units::si::ampere>(p, i, {}, 10.f);
  }

  void set_phases(uvh i)
  {
    using namespace mp_units;
    auto normalize = [this](auto i) -> float {
      return (i / i_max).numerical_value_in(one);
    };
    motor->u.set_duty_tristate(saturate(normalize(i.u), max_duty));
    motor->v.set_duty_tristate(saturate(normalize(i.v), max_duty));
    motor->h.set_duty_tristate(saturate(normalize(i.h), max_duty));
  }

  void loop(milliseconds dt, radian rotor, dq0 dq_current)
  {
    dq0 output;
    output.d = d_pid.loop(dt, dq_current.d);
    output.q = q_pid.loop(dt, dq_current.q);

    set_phases(inverse_clark_park(output, rotor));
  }

  void set_torque(amps current)
  {
    q_pid.target = current;
  }

private:
  // todo fix this to strong_ptr
  triple_hbridge* motor;

  zero_pi_controller<mp_units::si::ampere> d_pid;
  pi_controller<mp_units::si::ampere> q_pid;

  amps i_max;
  float max_duty;
};

}  // namespace foc
