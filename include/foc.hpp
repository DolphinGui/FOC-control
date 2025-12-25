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

"Sensorless" means there is no encoder on the rotor.
"Open loop" means there is no current sensor, "Closed loop" requires
a current sensor.

Use sensorless if the rotor encoder is too inaccurate, too slow, or doesn't
exist. Use open loop if there are no current measurement sensors installed.

*/
namespace foc {
struct abc
{
  amps a, b, c;
};

struct abc_duty
{
  uint16_t a, b, c;
};

struct dq0
{
  amps d, q;
};

// todo maybe change to a unit generic function??
inline dq0 clark_park(abc i, radian theta) noexcept
{
  using namespace mp_units;
  using namespace mp_units::si::unit_symbols;

  /* Park-clark transform is using pre-multiplied cosine/sine operations
  assuming cosine is really expensive (todo search up lut based cosine maybe)
  Matrix: [ sqrt(2/3) cos, -1/sqrt(6) cos + 1/sqrt(2) sin, -1/sqrt(6) cos -
  1/sqrt(2) sin ] [ -sqrt(2/3) sin, 1/sqrt(6) cos + 1/sqrt(2) sin, 1/sqrt(6) sin
  - 1/sqrt(2) cos ] 1/sqrt(3) [ 1, 1, 1 ]
  */
  quantity cs = si::cos(theta);
  quantity sn = si::sin(theta);

  quantity d = (sqrt(2.0f / 3.0f) * cs) * i.a +
               (-cs / sqrt(6.0f) + sn / sqrt(2.0f)) * i.b +
               (-cs / sqrt(6.0f) - sn / sqrt(2.0f)) * i.c;

  quantity q = (-sqrt(2.0f / 3.0f) * sn) * i.a +
               (cs / sqrt(6.0f) + sn / sqrt(2.0f)) * i.b +
               (sn / sqrt(6.0f) - cs / sqrt(2.0f)) * i.c;

  // quantity z = (i.a + i.b + i.c) / sqrt(3.0);

  return dq0{ d, q };
}

// todo fix to reduce trig function usage if this proves to be an issue
inline abc inverse_clark_park(dq0 i, radian theta) noexcept
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
inline abc_duty space_vector(dq0 i, radian theta, amps max_current)
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
      return abc_duty{ t02,
                       static_cast<uint16_t>(t02 + ta),
                       static_cast<uint16_t>(ts - t02) };
    case 1:
      return abc_duty{ static_cast<uint16_t>(t02 + tb),
                       t02,
                       static_cast<uint16_t>(ts - t02) };
    case 2:
      return abc_duty{ static_cast<uint16_t>(ts - t02),
                       t02,
                       static_cast<uint16_t>(t02 + ta) };
    case 3:
      return abc_duty{ static_cast<uint16_t>(ts - t02),
                       static_cast<uint16_t>(t02 + tb),
                       t02 };
    case 4:
      return abc_duty{ static_cast<uint16_t>(t02 + ta),
                       static_cast<uint16_t>(ts - t02),
                       t02 };
    case 5:
      return abc_duty{ t02,
                       static_cast<uint16_t>(ts - t02),
                       static_cast<uint16_t>(t02 + tb) };
    default:
      std::terminate();
  }
}

template<bool is_sensored>
struct foc
{

  void set_phases(abc i)
  {
    using namespace mp_units;
    auto normalize = [this](auto i) -> float {
      return (i / max_current).numerical_value_in(one);
    };
    pwm_a.set_duty_tristate(saturate(normalize(i.a), max_duty));
    pwm_b.set_duty_tristate(saturate(normalize(i.b), max_duty));
    pwm_c.set_duty_tristate(saturate(normalize(i.c), max_duty));
  }

  void loop(miliseconds dt)
  {
    using namespace mp_units;
    using namespace mp_units::si;
    radian rotor;
    if constexpr (is_sensored) {
      rotor = rotor_encoder.get_angle();
    } else {
      rotor = estimate_angle();
    }

    dq0 output;
    if (is_closed_loop) {
      amps I_a = curr_a.get_current(), I_b = curr_b.get_current(),
           I_c = curr_c.get_current();
      dq0 dq_current = clark_park({ I_a, I_b, I_c }, rotor);

      output.d = d_pid.loop(dt, dq_current.d);
      output.q = q_pid.loop(dt, dq_current.q);
    } else {
      output.d = 0.0f * ampere;
      output.q = torque_target;
    }

    set_phases(inverse_clark_park(output, rotor));
  }

  void set_torque(amps current)
  {
    if (is_closed_loop) {
      q_pid.target = current;
    } else {
      torque_target = current;
    }
  }

private:
  radian estimate_angle();

  hbridge pwm_a, pwm_b, pwm_c;
  current_sensor curr_a, curr_b, curr_c;

  tmp::maybe<is_sensored, encoder> rotor_encoder;

  float max_duty;

  zero_pi_controller<mp_units::si::ampere> d_pid;
  pi_controller<mp_units::si::ampere> q_pid;
  amps torque_target;

  amps max_current;
  bool is_closed_loop;
};

}  // namespace foc
