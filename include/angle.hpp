#pragma once

#include <tuple>

#include "utility.hpp"
#include "vectors.hpp"
#include <mp-units/framework/dimension.h>
#include <mp-units/systems/angular/units.h>
#include <mp-units/systems/si/math.h>

namespace foc {

namespace mp = mp_units;
namespace si = mp_units::si;

inline ab<V> clarke(uvh<V> v) noexcept
{
  volts a = sqrtf(2.0f / 3.0f) * (v.u - 0.5f * v.v - 0.5f * v.h);
  volts b = sqrtf(2.0f) / 2.0f * (v.v - v.h);
  return { a, b };
}

// algorithm based on
// https://davidmolony.github.io/MESC_Firmware/operation/CONTROL.html#the-sensorless-observer
struct flux_linkage_observer
{
  constexpr flux_linkage_observer(motor_characteristics m)
    : v_in(m.v_in)
    , r_m(m.phase_resistance)
    , l_m(m.phase_inductance)
  {
    // https://ww2.mathworks.cn/help/sps/ug/parameterize-a-permanent-magnet-synchronous-motor.html
    // for some reason simpleFOC adds a sqrt(3) coefficient, need to test to see
    // if this is accurate
    using namespace mp_units;
    using namespace mp_units::si::unit_symbols;
    flux_linkage = 1 / (m.kv / rad * m.pole_pairs);
  }

  radians estimate_angle(milliseconds dt, triple_hbridge const& motor, ab<A> i)
  {
    using namespace mp_units::si::unit_symbols;
    ab<V> v_m = clarke({ v_in * motor.u.get_duty_tristate(),
                         v_in * motor.v.get_duty_tristate(),
                         v_in * motor.h.get_duty_tristate() });
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

// based on https://doi.org/10.1016/j.aej.2021.12.005
struct hfi_observer
{

  // todo: extensive modelling regarding phase gain and stuff
  // using only motor characteristics
  hfi_observer(motor_characteristics m)
    : lp_filter(1 * mS, injection_period)
  {
    r4 = pow<4>(m.phase_resistance);
    r2w2 = pow<2>(m.phase_resistance) * pow<2>(injection_frequency);
  }
  /*
  HFI, despite its name, only injects pulses at low frequencies (~1kHz),
  but when the motor is moving at a slow rate (< 100 rotations per second)
  the bEMF is too small to meaningfully interfere with hfi.

  HFI pulses are injected on the d axis of dq0 to reduce torque disturbances.

  */
  std::pair<volts, radians> loop(milliseconds dt, ab<A> current)
  {
    using namespace mp_units::si::unit_symbols;
    using namespace mp_units;
    auto i = park(current, rotor);
    time += dt;
    rotor += dt * speed_err;
    float dut = (time / injection_period).numerical_value_in(one);
    if (dut < duty) {
      // start of period
      prev_i = i;
      sampled = false;
      begin_sample = time;
      return { injection_mag, rotor };
    } else if (dut < 0.5f) {
      // end of positive pulse
      if (!sampled && dut < duty + sample_delay) {
        di = i - prev_i;
        i_end = i;
        delta_t = time - begin_sample;
        sampled = true;
      }
    } else if (dut < 0.5f + duty) {
      prev_i = i;
      sampled = false;
      begin_sample = time;
      return { -injection_mag, rotor };
    } else if (dut < 0.5f + duty + sample_delay) {
      if (!sampled) {
        dq0<A> neg_di = i - prev_i;
        positive = abs(di.d) > abs(neg_di.d);
        if (!positive) {
          di = { neg_di.d, neg_di.q };
          i_end = i;
          delta_t = time - begin_sample;
        }
        sampled = true;
      }
    } else if (dut < 1.0f) {
      // just polled after pulse is finished
      if (!calculated) {
        update_controller(dt, estimate_err(dt));
        calculated = true;
      }
    } else {
      time = 0 * s;
      calculated = false;
    }
    return { 0 * V, rotor };
  }

  void update(milliseconds dt, radians r)
  {
    auto e = r - rotor;
    rotor = r;
    update_controller(dt, e);
  }

  mp::quantity<si::radian / mS, float> get_speed_err() const
  {
    return speed_err;
  }

private:
  constexpr static auto H = mp_units::si::henry;
  static dq0<H> estimate_inductance(milliseconds dt, dq0<A> di)
  {
    using namespace mp_units::si::unit_symbols;
    using namespace mp_units;
    // i = V/R(1-exp(-Rt/L))
    // di/dt = V/R * R/L exp(-Rt/L)
    //       = V/L * exp(-Rt/L)
    // L = V / di/dt
    // when t ~ 0, exp(-Rt/L) ~ 1
    // L ~ V / di/dt
    auto di_dt = (di / dt);
    quantity<H, float> d = injection_mag / abs(di_dt.d);
    quantity<H, float> q = injection_mag / abs(di_dt.q);
    return { d, q };
  }

  radians estimate_err(milliseconds dt)
  {
    auto L = estimate_inductance(dt, di);
    auto delta_l = (L.d - L.q) / 2;
    auto ld2 = mp::pow<2>(L.d);
    auto lq2 = mp::pow<2>(L.q);

    auto zmag2 = 2 * mp::sqrt(r4 + 2 * r2w2 * (ld2 + lq2) + w4 * ld2 * lq2);
    ohms delta_Z = delta_l * injection_frequency;
    amps i_hat = injection_mag * delta_Z / zmag2;
    auto e = si::asin(i_end.q / i_hat);
    if (!positive) {
      e += 180.0f * si::degree;
    }
    return e;
  }

  void update_controller(milliseconds dt, radians error)
  {
    speed_err = err_controller.loop(dt, lp_filter.loop(error));
  }

  constexpr static mp::quantity<si::radian / si::second, float>
    injection_frequency = 1000 * 2 * M_PI * si::radian / si::second;
  // this is stupid but pow is not constexpr so bite me
  constexpr static mp::quantity<mp::pow<4>(si::radian / si::second), float> w4 =
    injection_frequency * injection_frequency * injection_frequency *
    injection_frequency;

  constexpr static milliseconds injection_period =
    (2 * M_PI * si::radian) / (injection_frequency);

  constexpr static volts injection_mag = 10 * si::volt;

  constexpr static float duty = 0.01f;
  constexpr static float sample_delay = 0.001f;

  radians rotor{};
  dq0<A> prev_i{}, di{}, i_end{};
  milliseconds time{};
  milliseconds begin_sample;
  milliseconds delta_t;
  mp::quantity<si::radian / mS, float> speed_err{};
  mp::quantity<mp::pow<4>(si::ohm), float> r4{};
  mp::quantity<mp::pow<2>(si::ohm) / mp::pow<2>(mS), float> r2w2{};
  lowpass<si::radian> lp_filter;
  zero_pi_controller<si::radian, si::radian / mS> err_controller;
  bool positive = true;
  bool sampled = false;
  bool calculated = false;
};

}  // namespace foc