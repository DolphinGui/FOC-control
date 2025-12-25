#pragma once

#include "metaprogramming.hpp"
#include <mp-units/framework.h>
#include <mp-units/framework/quantity_cast.h>
#include <mp-units/systems/isq_angle.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/math.h>
#include <mp-units/systems/si/units.h>

#include <mp-units/math.h>

#include "metaprogramming.hpp"

namespace foc {
using amps = mp_units::quantity<mp_units::si::ampere, float>;
using radian = mp_units::quantity<mp_units::si::radian, float>;
using volts = mp_units::quantity<mp_units::si::volt, float>;
using ohms = mp_units::quantity<mp_units::si::ohm, float>;
using henries = mp_units::quantity<mp_units::si::henry, float>;
using milliseconds =
  mp_units::quantity<mp_units::si::milli<mp_units::si::second>, float>;
using rpmv = mp_units::quantity<mp_units::angular::revolution /
                                  mp_units::si::minute / mp_units::si::volt,
                                float>;
using rpm =
  mp_units::quantity<mp_units::angular::revolution / mp_units::si::minute,
                     float>;

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

template<auto R1, typename Rep1, auto R2, typename Rep2, auto R3, typename Rep3>
  requires requires(Rep1 v1, Rep2 v2, Rep3 v3) {
    get_common_reference(R1, R2);
    requires requires { remquo(v1, v2, &v3); } ||
               requires { std::remquo(v1, v2, &v3); };
  }
[[nodiscard]] constexpr mp_units::QuantityOf<get_quantity_spec(R1)> auto remquo(
  mp_units::quantity<R1, Rep1> const& x,
  mp_units::quantity<R2, Rep2> const& y,
  mp_units::quantity<R3, Rep3>* quo) noexcept
{
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = get_unit(ref);
  constexpr auto ref2 = get_common_reference(R3, R1 / R2);
  constexpr auto unit2 = mp_units::get_unit(ref2);
  using std::remquo;
  return mp_units::quantity{ remquo(x.numerical_value_in(unit),
                                    y.numerical_value_in(unit),
                                    &quo->numerical_value_ref_in(unit2)),
                             ref };
}

struct uvh
{
  amps u, v, h;
};

struct ab
{
  amps a, b;
};

struct current_sensor
{
  amps get_current();
};

struct motor_characteristics
{
  ohms phase_resistance;
  henries phase_inductance;
  rpmv kv;
  unsigned pole_pairs;
};

struct hbridge
{
  // positive values PWM the high side,
  // negative values PWM the low side, 0
  // does nothing.
  void set_duty_tristate(float);

  // does PWM where it is high until
  // it reaches a threshold value, where
  // it goes low. Incapable of
  void set_duty(uint16_t);

  float get_duty_tristate() const;
};

struct triple_hbridge
{
  hbridge u, v, h;
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
  pid_controller_t() = default;
  pid_controller_t(float kp,
                   maybe_float<is_I> ki = {},
                   maybe_float<is_D> kd = {},
                   maybe_float<is_I> max = {})
    : k_p(kp)
    , k_i(ki)
    , k_d(kd)
    , i_max(max)
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

  void reset()
  {
    total_err = {};
    prev_err = {};
  }

  void configure(float kp,
                 maybe_float<is_I> ki = {},
                 maybe_float<is_D> kd = {},
                 maybe_float<is_I> max = {})
  {
    k_p = kp;
    k_i = ki;
    k_d = kd;
    i_max = max;
    reset();
  }

  float k_p{};
  maybe_float<is_I> k_i{};
  maybe_float<is_D> k_d{};
  unit target{};
  maybe_float<is_I> i_max{};

private:
  maybe_float<is_I> total_err{};
  maybe_float<is_D> prev_err{};
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