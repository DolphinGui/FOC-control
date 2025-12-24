#pragma once

#include "metaprogramming.hpp"
#include <mp-units/framework/quantity_cast.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/math.h>
#include <mp-units/systems/si/units.h>

#include <mp-units/math.h>

#include "metaprogramming.hpp"

namespace foc {
using amps = mp_units::quantity<mp_units::si::ampere, float>;
using radian = mp_units::quantity<mp_units::si::radian, float>;

// saturates at [-bounds, bounds]
template<typename T = float>
inline T saturate(T f, T bounds)
{
  if (f > bounds)
    return bounds;
  if (f < -bounds)
    return -bounds;
  return f;
}

struct current_sensor
{
  amps get_current();
};

struct phase_pwm
{
  void set_duty(float);
};

struct encoder
{
  radian get_angle();
};

// pid_controller really needs to be compiled with -ffast-math, because
// on ARM without fpu, with const_target=0 a negation is not necessarily
// "safe". The precision isn't really necessary anyways for PID operations
template<auto R,
         bool is_I = true,
         bool is_D = true,
         tmp::optional<float> const_target = {}>
struct pid_controller_t
{
  using unit = mp_units::quantity<R, float>;
  template<bool b>
  using maybe_float = tmp::maybe<b, float>;
  template<bool b>
  using maybe_unit = tmp::maybe<b, unit>;
  using maybe_unit_int =
    tmp::maybe<is_I, mp_units::quantity<R * mp_units::si::second, float>>;

  using time = mp_units::quantity<mp_units::si::second, float>;
  pid_controller_t(float kp,
                   maybe_float<is_I> ki = {},
                   maybe_float<is_D> kd = {},
                   maybe_float<is_I> max = {})
    : k_p(kp)
    , k_i(ki)
    , k_d(kd)
    , target{}
    , i_max(max)
    , total_err{}
    , prev_err{}
  {
  }

  unit loop(time delta_time, unit current)
  {
    using namespace mp_units::si;
    float err;
    float dt = delta_time.numerical_value_in(second);
    if constexpr (const_target.exists) {
      err = const_target.value - current.numerical_value_in(R);
    } else {
      err = (target - current).numerical_value_in(R);
    }
    float result = k_p * err;
    if constexpr (is_I) {
      total_err = saturate(total_err + (err * dt), i_max);
      result += k_i * total_err;
    }

    if constexpr (is_D) {
      float change_err = (err - prev_err) / dt;
      result += k_d * change_err;
      prev_err = err;
    }

    return result * R;
  }

  float k_p;
  maybe_float<is_I> k_i;
  maybe_float<is_D> k_d;
  unit target;
  maybe_float<is_I> i_max;

private:
  maybe_float<is_I> total_err;
  maybe_float<is_D> prev_err;
};

template<auto R>
using pid_controller = pid_controller_t<R>;
template<auto R>
using pi_controller = pid_controller_t<R, true, false>;
template<auto R>
using zero_pi_controller = pid_controller_t<R, true, false>;

inline void test_pid()
{
  using namespace mp_units::si;
  using namespace mp_units::si::unit_symbols;
  pid_controller<rad> pid(1.0, 1.0, -1.0, 10.0);
  pid.target = 180 * deg;
  auto out = pid.loop(0.10 * ms, 0 * deg);
}
}  // namespace foc