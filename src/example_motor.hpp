#pragma once

#include "angle.hpp"
#include "foc.hpp"
#include "utility.hpp"
#include <chrono>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>
#include <mp-units/framework.h>
#include <mp-units/systems/angular/units.h>
#include <mp-units/systems/si/units.h>

enum struct control_mode : uint8_t
{
  pos,
  vel,
  accel
};

using namespace mp_units::si::unit_symbols;
using namespace foc::unit_symbols;
struct sensorless_motor
{
  constexpr static auto rpm = rev / min;
  constexpr static auto rpmv = rev / min / V;
  // based on
  // https://www.cubemars.com/product/ak60-39-v3-0-kv80-robotic-actuator.html
  constexpr static foc::motor_characteristics characteristics =
    foc::motor_characteristics{ .v_in = 40 * V,
                                .phase_resistance = 0.300f * Ω,
                                .phase_inductance = 0.335f * H,
                                .kv = 80.0f * rpmv,
                                .pole_pairs = 14 };

  constexpr static auto kt = 0.12f * N * m / A;

  constexpr static float gear_ratio = 39.0f;

  void loop(std::chrono::duration<float> delta_time)
  {
    using namespace mp_units::si::unit_symbols;
    using namespace mp_units::si;
    using namespace mp_units;
    quantity<milli<second>, float> dt = delta_time;
    auto i = shunts.get_current();
    foc::ab a = foc::clarke(i);
    foc::radians rotor;
    foc::volts injection_voltage{};
    if (slow) {
      auto [v, r] = hfi.loop(dt, foc::park(a, hfi.get_angle()).q);
      injection_voltage = v;
      rotor = r;
    } else {
      rotor = observer.estimate_angle(dt, hbridge, a);
      hfi.update(dt, rotor);
    }
    if (iteration > 100) {
      iteration = 0;
      auto v = (rotor - prev_angle) / dt;
      if (slow) {
        v -= hfi.get_speed_err();
      }
      slow = v < 628 * rad / s;
      switch (mode) {
        case control_mode::pos:
          w_pid.target = theta_pid.loop(dt, output_encoder.get_angle());
          [[fallthrough]];
        case control_mode::vel:
          torque_controller.set_torque(w_pid.loop(dt, v));
          prev_angle = rotor;
          [[fallthrough]];
        case control_mode::accel:
          break;
      }
    }
    torque_controller.loop(dt, rotor, foc::park(a, rotor), injection_voltage);
    iteration += 1;
  }

  void set_pos(foc::radians angle)
  {
    mode = control_mode::pos;
    theta_pid.target = angle;
  }

  void set_vel(mp_units::quantity<rpm, float> vel)
  {
    mode = control_mode::vel;
    w_pid.target = vel * gear_ratio;
  }

  void set_torque(mp_units::quantity<N * m, float> t)
  {
    mode = control_mode::accel;
    torque_controller.set_torque(t / kt);
  }

  sensorless_motor create(hal::steady_clock& clk)
  {
    foc::triple_hbridge hbridge;
    foc::triple_current_sensor shunts;
    auto [r, s] =
      foc::hfi_observer::initialization(clk, hbridge, shunts, characteristics);
    foc::hfi_observer hfi(characteristics, s, r, 100 * us);

    return sensorless_motor(std::move(hbridge), std::move(shunts), {}, r, s);
  }

private:
  sensorless_motor(foc::triple_hbridge&& h,
                   foc::triple_current_sensor&& i,
                   foc::encoder&& e,
                   foc::radians rotor,
                   foc::amps saliency)
    : hbridge(std::move(h))
    , shunts(std::move(i))
    , output_encoder(std::move(e))
    , torque_controller(characteristics, 2730 * rpm, 0.1125f, &hbridge)
    , observer(characteristics)
    , hfi(characteristics, saliency, rotor, 100 * us)
    , theta_pid(1.0f / ms, 0.1f / ms / s)
    , w_pid(1.0f * A * ms, 0.1f * A * ms / s)
    , mode(control_mode::pos)
  {
  }

  foc::triple_hbridge hbridge;
  foc::triple_current_sensor shunts;
  foc::encoder output_encoder;
  foc::closed_loop_controller torque_controller;
  foc::flux_linkage_observer observer;
  foc::hfi_observer hfi;

  foc::pid_controller<rad, rad / ms> theta_pid;
  foc::pid_controller<rad / ms, A> w_pid;
  mp_units::quantity<rad, float> prev_angle;
  unsigned iteration{};
  control_mode mode;
  bool slow = true;
};