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

struct sensorless_motor
{
  constexpr static auto rpm =
    mp_units::angular::revolution / mp_units::si::minute;
  constexpr static auto rpmv =
    mp_units::angular::revolution / mp_units::si::minute / mp_units::si::volt;
  // based on
  // https://www.cubemars.com/product/ak60-39-v3-0-kv80-robotic-actuator.html
  constexpr static foc::motor_characteristics characteristics =
    foc::motor_characteristics{ .v_in = 40 * mp_units::si::volt,
                                .phase_resistance = 0.300f * mp_units::si::ohm,
                                .phase_inductance =
                                  0.335f * mp_units::si::henry,
                                .kv = 80.0f * rpmv,
                                .pole_pairs = 14 };

  sensorless_motor()
    : controller(characteristics, 2730 * rpm, 0.1125f, &hbridge)
    , observer(characteristics)
    , hfi(characteristics)
    , mode(control_mode::pos)
  {
  }

  // There are an infinite amount of very clever and interesting ways to startup
  // a motor involving hfi, alignment, etc... These ways are too hard to think
  // about and don't matter too much anyways. The 39:1 gear reduction means even
  // a 180 degree realignment will result in a 5 degree jitter on startup.

  void startup(hal::steady_clock& clk)
  {
    hbridge.u.set_duty_tristate(0.05f);
    using namespace std::chrono_literals;
    hal::delay(clk, 100ms);
    hbridge.u.set_duty_tristate(0.0f);
  }

  void loop(std::chrono::duration<float> delta_time)
  {
    using namespace mp_units::si::unit_symbols;
    using namespace mp_units::si;
    using namespace mp_units;
    quantity<milli<second>, float> dt = delta_time;
    foc::uvh i{ u.get_current(), v.get_current(), h.get_current() };
    foc::ab a = foc::clarke(i);
    foc::radians rotor;
    foc::volts injection_voltage{};
    if (slow) {
      auto [v, r] = hfi.loop(dt, a);
      injection_voltage = v;
      rotor = r;
    } else {
      rotor = observer.estimate_angle(dt, hbridge, a);
      hfi.update(dt, rotor);
    }
    if (iteration > 100) {
      iteration = 0;
      auto v = (rotor - prev_angle) / dt;
      slow = v < 628 * rad / s;
      switch (mode) {
        case control_mode::pos:
          w_pid.target = theta_pid.loop(dt, rotor);
          [[fallthrough]];
        case control_mode::vel:
          controller.set_torque(w_pid.loop(dt, v));
          prev_angle = rotor;
          [[fallthrough]];
        case control_mode::accel:
          break;
      }
    }
    controller.loop(dt, rotor, foc::park(a, rotor), injection_voltage);
    iteration += 1;
  }

  void set_pos(foc::radians angle)
  {
  }

private:
  foc::triple_hbridge hbridge;
  foc::current_sensor u, v, h;
  foc::closed_loop_controller controller;
  foc::flux_linkage_observer observer;
  foc::hfi_observer hfi;

  constexpr static auto ang = mp_units::si::radian;
  constexpr static auto vel = mp_units::si::radian / mp_units::si::second;

  foc::pid_controller<ang, vel> theta_pid;
  foc::pid_controller<vel, foc::A> w_pid;
  mp_units::quantity<ang, float> prev_angle;
  unsigned iteration{};
  control_mode mode;
  bool slow = true;
};