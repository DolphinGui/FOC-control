#pragma once

// template metaprogramming
namespace tmp {

template<typename T>
struct returns_t
{
  using type = T;
};

struct null
{};

template<typename T>
struct optional
{
  bool exists = false;
  T value;
  constexpr optional() = default;
  constexpr optional(T t)
    : exists(true)
    , value(t)
  {
  }
};

template<bool, typename, typename>
struct if_else_t;

template<typename T, typename F>
struct if_else_t<true, T, F> : returns_t<T>
{};

template<typename T, typename F>
struct if_else_t<false, T, F> : returns_t<F>
{};

template<bool b, typename True, typename False>
using if_else = if_else_t<b, True, False>::type;


template<bool b, typename T>
using maybe = if_else<b, T, null>;
}  // namespace tmp