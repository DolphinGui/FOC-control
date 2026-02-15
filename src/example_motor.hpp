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

  void loop(std::chrono::duration<float> delta_time);

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

  static sensorless_motor create(foc::triple_hbridge* hbridge,
                                 foc::triple_current_sensor* shunts,
                                 foc::encoder* encoder)
  {
    // unsure if necessary, check later
    //   auto [r, s] =
    //     foc::hfi_observer::initialization(clk, *hbridge, shunts,
    //     characteristics);
    foc::hfi_observer hfi(0.0 * mp_units::si::radian,
                          1.0 * mp_units::si::ampere);

    return sensorless_motor(
      hbridge, shunts, encoder, {}, 1.0f * mp_units::si::ampere);
  }

private:
  sensorless_motor(foc::triple_hbridge* h,
                   foc::triple_current_sensor* i,
                   foc::encoder* e,
                   foc::radians rotor,
                   foc::amps saliency)
    : hbridge(h)
    , shunts(i)
    , output_encoder(e)
    , torque_controller(characteristics, 2730 * rpm, hbridge)
    , observer(characteristics)
    , hfi(rotor, saliency)
    , theta_pid(1.0f / ms, 0.1f / ms / s)
    , w_pid(1.0f * A * ms, 0.1f * A * ms / s)
    , mode(control_mode::pos)
  {
  }

  foc::triple_hbridge* hbridge;
  foc::triple_current_sensor* shunts;
  foc::encoder* output_encoder;
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