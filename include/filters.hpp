#pragma once

#include "utility.hpp"
#include <array>

namespace foc {

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

template<auto R>
struct lowpass
{
  constexpr lowpass(milliseconds time_constant, milliseconds sample_period)
  {
    a = sample_period / (time_constant + sample_period);
  }
  mp_units::quantity<R, float> loop(mp_units::quantity<R, float> x)
  {
    auto y = a * x + (1 - a) * prev;
    prev = y;
    return y;
  }

  mp_units::quantity<R, float> operator()(mp_units::quantity<R, float> x)
  {
    return loop(x);
  }

private:
  mp_units::quantity<R, float> prev;
  mp_units::quantity<mp_units::one, float> a;
};

struct biquad_coef
{
  std::array<float, 3> b;
  std::array<float, 2> a;
};

template<auto R, biquad_coef coef>
struct biquad_filt
{
  constexpr biquad_filt() = default;

  mp_units::quantity<R, float> loop(mp_units::quantity<R, float> x)
  {
    mp_units::quantity<R, float> y = x * coef.b[0];
    for (size_t i = 0; i < 2; ++i) {
      y += xs[i] * coef.b[i + 1] + ys[i] * coef.a[i];
    }
    for (int i = 1; i >= 0; --i) {
      xs[i] = xs[i - 1];
      ys[i] = ys[i - 1];
    }
    xs[0] = x;
    ys[0] = y;

    return y;
  }

  mp_units::quantity<R, float> operator()(mp_units::quantity<R, float> x)
  {
    return loop(x);
  }

private:
  std::array<mp_units::quantity<R, float>, 2> xs;
  std::array<mp_units::quantity<R, float>, 2> ys;
};

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