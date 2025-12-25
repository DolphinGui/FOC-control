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
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = get_unit(ref);
  return uvh<unit>{ vec.u / scalar, vec.v / scalar, vec.h / scalar };
}
template<auto R1, auto R2>
auto operator/(mp_units::quantity<R1, float> scalar, uvh<R2> vec)
{
  constexpr auto ref = get_common_reference(R1, R2);
  constexpr auto unit = get_unit(ref);
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

}  // namespace foc