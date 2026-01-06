#pragma once

#include <mp-units/framework.h>
#include <mp-units/framework/quantity_cast.h>
#include <mp-units/systems/isq_angle.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/math.h>
#include <mp-units/systems/si/units.h>

#include <mp-units/math.h>

namespace foc {
// mp_units does not have an si-compatible revolution, so here it is
inline constexpr struct revolution final
  : mp_units::named_unit<"rev",
                         mp_units::mag<2> * mp_units::mag<mp_units::π> *
                           mp_units::si::radian>
{
} revolution;
namespace unit_symbols {
constexpr static auto rev = revolution;
}
using amps = mp_units::quantity<mp_units::si::ampere, float>;
using radians = mp_units::quantity<mp_units::si::radian, float>;
using volts = mp_units::quantity<mp_units::si::volt, float>;
using ohms = mp_units::quantity<mp_units::si::ohm, float>;
using henries = mp_units::quantity<mp_units::si::henry, float>;
using milliseconds =
  mp_units::quantity<mp_units::si::milli<mp_units::si::second>, float>;
using rpmv =
  mp_units::quantity<revolution / mp_units::si::minute / mp_units::si::volt,
                     float>;
using rpm = mp_units::quantity<revolution / mp_units::si::minute, float>;

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

struct motor_characteristics
{
  volts v_in;
  ohms phase_resistance;
  henries phase_inductance;
  rpmv kv;
  unsigned pole_pairs;
};

template<auto U, typename Repr = float>
struct uvh;

}  // namespace foc