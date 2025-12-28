#pragma once

#include <libhal-util/steady_clock.hpp>
#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>
#include <stdexcept>
#include <tuple>

#include "filters.hpp"
#include "utility.hpp"
#include "vectors.hpp"
#include <libhal/steady_clock.hpp>
#include <mp-units/framework/dimension.h>
#include <mp-units/systems/angular/units.h>
#include <mp-units/systems/si.h>

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
  using saliency = mp_units::quantity<mp_units::si::ampere>;
  /*
  Saliency, funnily enough, is just a constant that is a part of the
  motor characteristics. This means that saliency is in effect swallowed
  by the PI controller, so just use a best effort guess of what saliency
  should be and tune from there.
  */
  hfi_observer(motor_characteristics const& m,
               saliency k_s,
               radians rotor,
               milliseconds average_dt)
    : rotor(rotor)
    , k_s(k_s)
    , lp_filter(injection_period, average_dt)
    // this is a dumb heruistic that hopefully won't blow up
    , err_controller(1.0f / mS, 0.1f / mS / si::second)
  {
  }

  static std::pair<radians, saliency> initialization(
    hal::steady_clock& clk,
    triple_hbridge& motor,
    triple_current_sensor shunts,
    motor_characteristics const& c)
  {
    using namespace mp_units;
    using namespace mp_units::si::unit_symbols;
    using namespace std::chrono_literals;
    auto rotor = initial_angle(clk, motor, shunts, c);
    // e^6 ~ 99.%, this is probably good enough
    auto settle_time = mp_units::to_chrono_duration(
      (c.phase_inductance / c.phase_resistance * 6.0f).force_in<hal::u64>(ns));
    // V/L * e^(-Rt/L) ~= V/L when -Rt/L ~= 0, find when e^(-Rt/L) = 99% and
    // determine sample time based on motor characteristics
    auto dt = (-c.phase_inductance / c.phase_resistance * log(0.99));
    auto sample_time = mp_units::to_chrono_duration(dt.force_in<hal::u64>(ns));

    // settle stator
    motor.set_duty({});
    hal::delay(clk, settle_time);

    // auto f = clk.frequency() * Hz;

    // cut off the current and measure settle time
    auto const measure_settle = [&]() {
      motor.set_duty({});
      auto start = clk.uptime();
      auto i = park(clarke(shunts.get_current()), rotor).d;
      while (abs(i) > 50 * mA) {
        hal::delay(clk, 100us);
        i = park(clarke(shunts.get_current()), rotor).d;
      }
      return clk.uptime() - start;
    };

    // sample positive inductance
    motor.set_duty(inverse_clark_park(dq0<one>{ 0.20f * one, {} }, rotor));
    hal::delay(clk, settle_time);
    auto t_p = measure_settle();

    motor.set_duty({});
    hal::delay(clk, settle_time);

    // sample negative inducatance
    motor.set_duty(inverse_clark_park(dq0<one>{ -0.20f * one, {} }, rotor));
    hal::delay(clk, settle_time);
    auto t_n = measure_settle();

    motor.set_duty({});
    hal::delay(clk, settle_time);

    // sample inductance of d-axis
    motor.set_duty(inverse_clark_park(dq0<one>{ 0.20f * one, {} }, rotor));
    hal::delay(clk, sample_time);
    auto i_d = park(clarke(shunts.get_current()), rotor).d;
    auto L_d = c.v_in / (i_d / dt);

    motor.set_duty({});
    hal::delay(clk, settle_time);

    // sample inductance of q-axis
    motor.set_duty(inverse_clark_park(dq0<one>{ {}, 0.20f * one }, rotor));
    hal::delay(clk, sample_time);
    auto i_q = park(clarke(shunts.get_current()), rotor).q;
    auto L_q = c.v_in / (i_q / dt);

    auto delta_L = (L_d - L_q) / 2;
    auto r_s = c.phase_resistance;
    auto w_h = injection_frequency;
    auto z_d = hypot(r_s, w_h * L_d);
    auto z_q = hypot(r_s, w_h * L_q);

    // see equation 12 of L. Sun
    auto salience = c.v_in * delta_L * w_h / (2 * z_d * z_q);

    motor.set_duty({});
    hal::delay(clk, settle_time);

    // 3.4, L. Sun
    if (t_p > t_n) {
      return { rotor, salience };
    } else {
      return { rotor + 180.0f * deg, salience };
    }
  }

  /*
  HFI, despite its name, only injects pulses at low frequencies (~1kHz),
  but when the motor is moving at a slow rate (< 100 rotations per second)
  the bEMF is too small to meaningfully interfere with hfi.

  HFI pulses are injected on the d axis of dq0 to reduce torque disturbances.
  */

  std::pair<volts, radians> loop(milliseconds dt, amps i_q)
  {
    time += dt;
    update_controller(
      dt, lp_filter(i_q * si::sin(time * injection_frequency) / k_s));
    rotor += dt * speed_err;
    auto voltage = si::cos(time * injection_frequency) * injection_mag;
    time = remainder(time, injection_period);
    return { voltage, rotor };
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

  void update_controller(milliseconds dt, radians error)
  {
    speed_err = err_controller.loop(dt, lp_filter.loop(error));
  }

  static radians initial_angle(hal::steady_clock& clk,
                               triple_hbridge& motor,
                               triple_current_sensor shunts,
                               motor_characteristics const& c)
  {
    using namespace mp_units;
    using namespace mp_units::si::unit_symbols;
    // This observer is probably suboptimally tuned, but that's ok
    // because this should be done on a stationary rotor
    hfi_observer hfi(c, 1.0f * A, 0.0f * rad, 10 * us);
    auto start = clk.uptime();
    auto last_time = start;
    auto f = clk.frequency() * Hz;
    // do this for at most 500 ms
    auto end = hal::u64(0.5f * clk.frequency()) + start;
    // todo change to a timer or something for timing consistency?
    radians rotor = {};
    using namespace std::chrono_literals;

    do {
      auto now = clk.uptime();
      auto dt = (now - last_time) / f;
      auto i = park(clarke(shunts.get_current()), rotor);
      auto [v, theta] = hfi.loop(dt, i.q);
      rotor = theta;
      auto duty = inverse_clark_park(dq0<V>{ v, {} }, rotor) / c.v_in;
      motor.set_duty(duty);
      hal::delay(clk, 10us);
    } while (last_time < end && hfi.get_speed_err() > 0.001f * rad / mS);
    // at this point the rotor should have converged, or we've given up.
    if (hfi.get_speed_err() >= 0.001f * rad / mS) {
      throw std::runtime_error(
        "Unable to determine initial rotor position using HFI");
    }
    return rotor;
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

  radians rotor;
  saliency k_s;
  milliseconds time{};
  mp::quantity<si::radian / mS, float> speed_err{};
  lowpass<si::radian> lp_filter;
  zero_pi_controller<si::radian, si::radian / mS> err_controller;
};

}  // namespace foc