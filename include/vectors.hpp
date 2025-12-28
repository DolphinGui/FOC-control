#pragma once

#include "utility.hpp"

#include "mp-units/framework.h"

namespace foc {

template<auto U>
struct uvh
{
  mp_units::quantity<U, float> u, v, h;
};

template<auto R1, auto R2>
auto operator*(uvh<R1> vec, mp_units::quantity<R2, float> scalar)
{
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = get_unit(ref);
  return uvh<unit>{ vec.u * scalar, vec.v * scalar, vec.h * scalar };
}
template<auto R1, auto R2>
auto operator*(mp_units::quantity<R2, float> scalar, uvh<R1> vec)
{
  return vec * scalar;
}

template<auto R1, auto R2>
auto operator/(uvh<R1> vec, mp_units::quantity<R2, float> scalar)
{
  constexpr auto unit = get_unit(R1 / R2);
  return uvh<unit>{ vec.u / scalar, vec.v / scalar, vec.h / scalar };
}
template<auto R1, auto R2>
auto operator/(mp_units::quantity<R1, float> scalar, uvh<R2> vec)
{
  constexpr auto unit = get_unit(R1 / R2);
  return uvh<unit>{ scalar / vec.u, scalar / vec.v, scalar / vec.h };
}

template<auto U>
struct ab
{
  mp_units::quantity<U, float> a, b;
  constexpr static bool is_2vec = true;
};

template<auto U>
struct dq0
{
  mp_units::quantity<U, float> d, q;
  constexpr static bool is_2vec = true;
};

template<typename T>
concept Vec2 = T::is_2vec;

template<auto R1, auto R2, template<auto> typename Vec>
  requires Vec2<Vec<R1>>
auto operator*(Vec<R1> vec, mp_units::quantity<R2, float> scalar)
{
  constexpr auto unit = get_unit(R1 * R2);
  auto [x, y] = vec;
  return Vec<unit>{ scalar * x, scalar * y };
}

template<auto R1, auto R2, template<auto> typename Vec>
  requires Vec2<Vec<R1>>
auto operator*(mp_units::quantity<R2, float> scalar, Vec<R1> vec)
{
  return vec * scalar;
}

template<auto R1, auto R2, template<auto> typename Vec>
  requires Vec2<Vec<R1>>
auto operator/(Vec<R1> vec, mp_units::quantity<R2, float> scalar)
{
  constexpr auto unit = get_unit(R1 / R2);
  auto [x, y] = vec;
  return Vec<unit>{ x / scalar, y / scalar };
}

template<auto R1, auto R2, template<auto> typename Vec>
  requires Vec2<Vec<R2>>
auto operator/(mp_units::quantity<R1, float> scalar, Vec<R2> vec)
{
  constexpr auto unit = get_unit(R1 / R2);
  auto [x, y] = vec;
  return Vec<unit>{ scalar / x, scalar / y };
}

template<auto R1, auto R2, template<auto> typename Vec>
  requires Vec2<Vec<R1>> && Vec2<Vec<R2>>
auto operator+(Vec<R1> v1, Vec<R2> v2)
{
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = get_unit(ref);
  auto [x, y] = v1;
  auto [a, b] = v2;
  return Vec<unit>{ x + a, y + b };
}
template<auto R1, auto R2, template<auto> typename Vec>
  requires Vec2<Vec<R1>> && Vec2<Vec<R2>>
auto operator-(Vec<R1> v1, Vec<R2> v2)
{
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = get_unit(ref);
  auto [x, y] = v1;
  auto [a, b] = v2;
  return Vec<unit>{ x - a, y - b };
}

template<auto R>
inline ab<R> clarke(uvh<R> i) noexcept
{
  amps a = sqrtf(2.0f / 3.0f) * (i.u - 0.5f * i.v - 0.5f * i.h);
  amps b = sqrtf(2.0f) / 2.0f * (i.v - i.h);
  return { a, b };
}

template<auto R>
inline dq0<R> park(ab<R> i, radians theta)
{
  auto c = mp_units::si::cos(theta);
  auto s = mp_units::si::sin(theta);
  return { c * i.a + s * i.b, -s * i.a + c * i.b };
}

// todo fix to reduce trig function usage if this proves to be an issue
template<auto R>
inline uvh<R> inverse_clark_park(dq0<R> i, radians theta) noexcept
{
  using namespace mp_units;
  using namespace mp_units::si::unit_symbols;
  radians phase_offset = 60 * si::degree;

  volts u = si::cos(theta) * i.d - si::sin(theta) * i.q;
  volts v =
    si::cos(theta - phase_offset) * i.q - si::sin(theta - phase_offset) * i.d;
  volts h =
    si::cos(theta + phase_offset) * i.q - si::sin(theta + phase_offset) * i.d;
  return { u, v, h };
}

template<auto R>
inline uvh<R> inverse_clarke(ab<R> i) noexcept
{
  using namespace mp_units;
  using namespace mp_units::si::unit_symbols;
  return { sqrtf(2.0f / 3.0f) * i.a,
           -1.0f / sqrtf(6.0f) * i.a + 1.0f / sqrtf(2.0f) * i.b,
           -1.0f / sqrtf(6.0f) * i.a - 1.0f / sqrtf(2.0f) * i.b };
}

inline void triple_hbridge::set_duty(uvh<mp_units::one> duty)
{
  u.set_duty(duty.u.numerical_value_in(mp_units::one));
  v.set_duty(duty.v.numerical_value_in(mp_units::one));
  h.set_duty(duty.h.numerical_value_in(mp_units::one));
}

inline uvh<mp_units::si::ampere> triple_current_sensor::get_current()
{
  return { u.get_current(), v.get_current(), w.get_current() };
}

}  // namespace foc