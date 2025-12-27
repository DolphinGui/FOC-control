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
#include "vectors.hpp"
#include "filters.hpp"
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

using uvh_v = uvh<V>;
using dq0_v = dq0<V>;
// todo fix to reduce trig function usage if this proves to be an issue
inline uvh_v inverse_clark_park(dq0_v i, radians theta) noexcept
{
  using namespace mp_units;
  using namespace mp_units::si::unit_symbols;
  radians phase_offset = 60 * si::degree;

  volts u = si::cos(theta) * i.d - si::sin(theta) * i.q;
  volts v =
    si::cos(theta - phase_offset) * i.q - si::sin(theta - phase_offset) * i.d;
  volts h =
    si::cos(theta + phase_offset) * i.q - si::sin(theta + phase_offset) * i.d;
  return uvh_v{ u, v, h };
}

// See table 2 of https://ww1.microchip.com/downloads/en/appnotes/00955a.pdf
// todo stare at assembly and maybe save 1 trig operation
inline uvh_duty space_vector(dq0<A> i, radians theta, amps max_current)
{
  using namespace mp_units::si::unit_symbols;
  using namespace mp_units;
  using duty = quantity<one, float>;

  radians ang = si::atan2(i.q, i.d) + theta;
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

struct closed_loop_controller
{
  // See
  // https://davidmolony.github.io/MESC_Firmware/operation/CONTROL.html#the-foc-pi
  // for gain calculations
  closed_loop_controller(motor_characteristics m,
                         rpm max_speed,
                         float max_duty,
                         triple_hbridge* bridge)
    : motor(bridge)
    , v_in(m.v_in)
    , max_duty(max_duty)
  {
    using namespace mp_units;
    using namespace mp_units::si::unit_symbols;
    auto p = (m.phase_inductance * max_speed / rad).in(V / A);
    auto i = (m.phase_resistance / m.phase_inductance).in(Hz)*ohm;
    // todo add saturation current
    d_pid = zero_pi_controller<A, V>(p, i, {}, 10.f * A * s);
    q_pid = pi_controller<A, V>(p, i, {}, 10.f * A * s);
  }

  void set_phases(uvh_v i)
  {
    using namespace mp_units;
    auto normalize = [this](auto v) -> float {
      return (v / v_in).numerical_value_in(one);
    };
    motor->u.set_duty_tristate(saturate(normalize(i.u), max_duty));
    motor->v.set_duty_tristate(saturate(normalize(i.v), max_duty));
    motor->h.set_duty_tristate(saturate(normalize(i.h), max_duty));
  }

  void loop(milliseconds dt,
            radians rotor,
            dq0<A> dq_current,
            volts d_inject = {})
  {
    dq0_v output;
    output.d = d_pid.loop(dt, dq_current.d) + d_inject;
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

  zero_pi_controller<mp_units::si::ampere, mp_units::si::volt> d_pid;
  pi_controller<mp_units::si::ampere, mp_units::si::volt> q_pid;

  volts v_in;
  float max_duty;
};

struct open_loop_controller
{
  open_loop_controller(motor_characteristics m,
                       float max_duty,
                       triple_hbridge* bridge)
    : motor(bridge)
    , i_max(m.v_in)
    , max_duty(max_duty)
    , phase_resistance(m.phase_resistance)
  {
  }

  void set_phases(uvh_v i)
  {
    using namespace mp_units;
    auto normalize = [this](auto i) -> float {
      return (i / i_max).numerical_value_in(one);
    };
    motor->u.set_duty_tristate(saturate(normalize(i.u), max_duty));
    motor->v.set_duty_tristate(saturate(normalize(i.v), max_duty));
    motor->h.set_duty_tristate(saturate(normalize(i.h), max_duty));
  }

  void loop(radians rotor)
  {
    dq0_v output{};
    output.q = target * 2 * phase_resistance;

    set_phases(inverse_clark_park(output, rotor));
  }

  void set_torque(amps current)
  {
    target = current;
  }

private:
  // todo fix this to strong_ptr
  triple_hbridge* motor;
  volts i_max;
  float max_duty;
  amps target;
  ohms phase_resistance;
};

}  // namespace foc
