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
  // a startup procedure is necessary to initialize rotor to 0 at beginning.
  // could do something clever with saturation negative edge resolution, but
  // for now we don't need to be clever and can just do dumb initilization
  hfi_observer() = default;

  volts inject(milliseconds dt)
  {
    using namespace mp_units::si::unit_symbols;
    volts injection = 0 * V;
    time_since_last_pulse += dt;
    if (time_since_last_pulse < duty * injection_period) {
      injection = injection_mag;
    } else if (time_since_last_pulse > injection_period) {
      time_since_last_pulse = 0 * s;
    }
    return injection;
  }

  /*
  HFI, despite its name, only injects pulses at low frequencies (~1kHz),
  but when the motor is moving at a slow rate (< 100 rotations per second)
  the bEMF is too small to meaningfully interfere with hfi.

  HFI pulses are injected on the d axis of dq0 to reduce torque disturbances.
  */
  std::pair<volts, radians> loop(milliseconds dt, dq0<A> i)
  {
    using namespace mp_units::si::unit_symbols;
    using namespace mp_units;
    time_since_last_pulse += dt;
    if (time_since_last_pulse < duty * injection_period) {
      prev_i = i;
      return { injection_mag, rotor };
    } else if (time_since_last_pulse < injection_period) {
      // just polled after pulse is finished
      auto L = estimate_inductance(dt, i - prev_i);
      auto mag = mp::hypot(L.q, L.d);
      auto ang = si::asin(L.q / mag);
      // The Sun paper seems? to be implementing a speed controller, and does
      // some weird stuff with output PI and lowpass filtering. Not really sure
      // how to do that but this should be good enough in theory.
      rotor = ang;
    } else {
      time_since_last_pulse = 0 * s;
    }
    return { 0 * V, rotor };
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
    quantity<H, float> d = injection_mag / di_dt.d;
    quantity<H, float> q = injection_mag / di_dt.q;
    return { d, q };
  }

  constexpr static mp::quantity<si::radian / si::second, float>
    injection_frequency = 1000 * 2 * M_PI * si::radian / si::second;

  constexpr static milliseconds injection_period =
    (2 * M_PI * si::radian) / (injection_frequency);

  constexpr static volts injection_mag = 10 * si::volt;
  constexpr static float duty = 0.01f;

  radians rotor{};
  dq0<A> prev_i{};
  milliseconds time_since_last_pulse{};
};

}  // namespace foc