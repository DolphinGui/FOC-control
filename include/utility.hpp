#pragma once

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
using radians = mp_units::quantity<mp_units::si::radian, float>;
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

constexpr static auto A = mp_units::si::ampere;
constexpr static auto V = mp_units::si::volt;
constexpr static auto mS = mp_units::si::milli<mp_units::si::second>;

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
  radians get_angle();
};

// pid_controller really needs to be compiled with -ffast-math, because
// on ARM without fpu, with const_target=0 a negation is not necessarily
// "safe". The precision isn't really necessary anyways for PID operations
template<auto In,
         auto Out,
         auto Time = mp_units::si::second,
         bool is_I = true,
         bool is_D = true,
         tmp::optional<mp_units::quantity<In, float>> const_target = {}>
struct pid_controller_t
{
  using time = mp_units::quantity<mp_units::si::second, float>;
  template<bool b, typename T>
  using maybe = tmp::maybe<b, T>;

  constexpr static auto gain = Out / In;
  constexpr static auto integrated_gain = Out / (In * Time);
  constexpr static auto integral = In * Time;
  constexpr static auto derivative_gain = Out / (In / Time);

  using gain_q = mp_units::quantity<gain, float>;
  using igain_q = mp_units::quantity<integrated_gain, float>;
  using dgain_q = mp_units::quantity<derivative_gain, float>;
  using integral_input = mp_units::quantity<integral, float>;

  pid_controller_t() = default;
  pid_controller_t(gain_q kp,
                   maybe<is_I, igain_q> ki = {},
                   maybe<is_D, dgain_q> kd = {},
                   maybe<is_I, integral_input> max = {})
    : k_p(kp)
    , k_i(ki)
    , k_d(kd)
    , i_max(max)
  {
  }

  mp_units::quantity<Out, float> loop(time dt,
                                      mp_units::quantity<In, float> current)
  {
    using namespace mp_units::si;
    using namespace mp_units;
    quantity<In, float> err;
    if constexpr (const_target.exists) {
      err = const_target.value - current;
    } else {
      err = target - current;
    }
    quantity result = k_p * err;
    if constexpr (is_I) {
      total_err = saturate(total_err + (err * dt), i_max);
      result += k_i * total_err;
    }

    if constexpr (is_D) {
      quantity change_err = (err - prev_err) / dt;
      result += k_d * change_err;
      prev_err = err;
    }

    return result * Out;
  }

  void reset()
  {
    total_err = {};
    prev_err = {};
  }

  void configure(gain_q kp,
                 maybe<is_I, igain_q> ki = {},
                 maybe<is_D, dgain_q> kd = {},
                 maybe<is_I, integral_input> max = {})
  {
    k_p = kp;
    k_i = ki;
    k_d = kd;
    i_max = max;
    reset();
  }

  gain_q k_p{};
  maybe<is_I, igain_q> k_i = {};
  maybe<is_D, dgain_q> k_d = {};
  maybe<!const_target.exists, mp_units::quantity<In, float>> target{};
  maybe<is_I, integral_input> i_max{};

private:
  maybe<is_I, integral_input> total_err{};
  maybe<is_D, mp_units::quantity<In, float>> prev_err{};
};

template<auto In, auto Out>
using pid_controller = pid_controller_t<In, Out>;
template<auto In, auto Out>
using pi_controller =
  pid_controller_t<In, Out, mp_units::si::second, true, false>;
template<auto In, auto Out>
using zero_pi_controller =
  pid_controller_t<In, Out, mp_units::si::second, true, false>;

[[maybe_unused]]
inline void test_pid()
{
  using namespace mp_units;
  using namespace mp_units::si;
  using namespace mp_units::si::unit_symbols;
  pid_controller<rad, rad> pid(
    1.0f * one, 1.0f / second, 1.0f * second, 10.0f * second * rad);
  pid.target = 180 * deg;
  [[maybe_unused]]
  auto out = pid.loop(0.10 * ms, 0 * deg);
}
}  // namespace foc