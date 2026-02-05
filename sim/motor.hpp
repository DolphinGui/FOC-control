#pragma once

#include <mp-units/systems/si.h>
#include <mp-units/systems/si/units.h>

namespace motor {

inline constexpr struct revolution final
  : mp_units::named_unit<"rev",
                         mp_units::mag<2> * mp_units::mag<mp_units::π> *
                           mp_units::si::radian>
{
} revolution;

using namespace mp_units;
using namespace mp_units::si;
struct motor_characteristics
{
  quantity<volt> v_in;
  quantity<ohm> phase_resistance;
  quantity<henry> phase_inductance;
  quantity<revolution / minute / volt> kv;
  quantity<newton * metre / ampere> kt;
  quantity<gram * pow<2>(metre)> moment;
  unsigned pole_pairs;
  unsigned slots;
};

template<auto U>
struct uvw
{
  mp_units::quantity<U> u, v, w;
};

struct motor
{
  motor_characteristics stats;
  quantity<radian> rotor_angle;
  quantity<radian / second> rotor_vel;
  uvw<ampere> stator_i;
  uvw<volt> stator_v;
};

}  // namespace motor