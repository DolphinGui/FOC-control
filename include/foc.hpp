#pragma once

#include <mp-units/framework/quantity_cast.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/math.h>
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

  quantity d = (sqrt(2.0 / 3.0) * cs) * i.a +
               (-cs / sqrt(6.0) + sn / sqrt(2.0)) * i.b +
               (-cs / sqrt(6.0) - sn / sqrt(2.0)) * i.c;

  quantity q = (-sqrt(2.0 / 3.0) * sn) * i.a +
               (cs / sqrt(6.0) + sn / sqrt(2.0)) * i.b +
               (sn / sqrt(6.0) - cs / sqrt(2.0)) * i.c;

  // quantity z = (i.a + i.b + i.c) / sqrt(3.0);

  return dq0{ d, q };
}

inline abc inverse_clark_park(dq0 i, radian theta) noexcept
{
  using namespace mp_units;
  using namespace mp_units::si::unit_symbols;
  quantity phase_offset = 2.0 / 3.0 * M_PI * si::radian;

  quantity a = si::cos(theta) * i.d - si::sin(theta) * i.q;
  quantity b =
    si::cos(theta - phase_offset) * i.q - si::sin(theta - phase_offset) * i.d;
  quantity c =
    si::cos(theta + phase_offset) * i.q - si::sin(theta + phase_offset) * i.d;
  return { a, b, c };
}
/* Space vector is nice because it uses far fewer trig operations
   and has higher power, but has less smooth commutation. */
inline abc space_vector(dq0 i, radian theta)
{
  using namespace mp_units::si::unit_symbols;
  using namespace mp_units;
  radian ang = si::atan2(i.q, i.d) + theta;
  amps mag = hypot(i.q, i.d);

  if (ang >= -30 * deg && ang < 30 * deg) {
    return abc{ mag, -mag / 2, -mag / 2 };
  } else if (ang >= 30 * deg && ang < 90 * deg) {
    return abc{ mag / 2, mag / 2, -mag };
  } else if (ang >= 90 * deg && ang < 150 * deg) {
    return abc{ -mag / 2, mag, -mag / 2 };
  } else if (ang >= 150 * deg && ang < 210 * deg) {
    return abc{ -mag, mag / 2, mag / 2 };
  } else if (ang >= 210 * deg && ang < 270 * deg) {
    return abc{ -mag / 2, -mag / 2, mag };
  } else {
    //(ang >= 270 * deg && ang < 330 * deg)
    return abc{ mag / 2, -mag, mag / 2 };
  }
}

template<bool is_closed_loop, bool is_sensored>
struct foc
{

  void set_phases(abc i)
  {
    using namespace mp_units;
    pwm_a.set_duty(saturate(i.a.numerical_value_in(si::ampere), max_duty));
    pwm_b.set_duty(saturate(i.b.numerical_value_in(si::ampere), max_duty));
    pwm_c.set_duty(saturate(i.c.numerical_value_in(si::ampere), max_duty));
  }

  void loop(float dt)
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
    if constexpr (is_closed_loop) {
      amps I_a = curr_a.get_current(), I_b = curr_b.get_current(),
           I_c = curr_c.get_current();
      dq0 dq_current = clark_park({ I_a, I_b, I_c }, rotor);

      output.d =
        d_pid.loop(dt, dq_current.d.numerical_value_in(si::ampere)) * ampere;

      output.q =
        q_pid.loop(dt, dq_current.q.numerical_value_in(si::ampere)) * ampere;
    } else {
      output.d = 0.0f * ampere;
      output.q = torque_target;
    }

    abc phases = inverse_clark_park(output, rotor);
    set_phases(phases);
  }

  void set_torque(amps current)
  {
    if constexpr (is_closed_loop) {
      q_pid.target = current;
    } else {
      torque_target = current;
    }
  }

private:
  radian estimate_angle();

  phase_pwm pwm_a, pwm_b, pwm_c;
  tmp::maybe<is_closed_loop, current_sensor> curr_a, curr_b, curr_c;

  tmp::maybe<is_sensored, encoder> rotor_encoder;

  float max_duty;

  tmp::maybe<is_closed_loop, zero_pi_controller<mp_units::si::ampere>> d_pid;
  tmp::maybe<is_closed_loop, pi_controller<mp_units::si::ampere>> q_pid;
  tmp::maybe<!is_closed_loop, amps> torque_target;
};

using open_sensored_foc = foc<false, true>;
using closed_sensorless_foc = foc<true, false>;

}  // namespace foc
