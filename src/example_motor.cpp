#include "example_motor.hpp"

void sensorless_motor::loop(std::chrono::duration<float> delta_time)
{
  using namespace mp_units::si::unit_symbols;
  using namespace mp_units::si;
  using namespace mp_units;
  quantity<milli<second>, float> dt = delta_time;
  auto i = shunts->get_current();
  foc::ab a = foc::clarke(i);
  foc::radians rotor;
  foc::volts injection_voltage{};
  if (slow) {
    auto [v, r] = hfi.loop(dt, foc::park(a, hfi.get_angle()).q);
    injection_voltage = v;
    rotor = r;
  } else {
    rotor = observer.estimate_angle(dt, *hbridge, a);
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
        w_pid.target = theta_pid.loop(dt, output_encoder->get_angle());
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