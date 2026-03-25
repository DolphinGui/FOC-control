#pragma once

#include <cstddef>

namespace util {
template<size_t len>
struct str_const
{
  char value[len]{};
  constexpr str_const() = default;
  template<size_t strlen>
  constexpr str_const(char const (&lit)[strlen]) noexcept
  {
    for (size_t i = 0; i != len; ++i) {
      value[i] = lit[i];
    }
  }
  template<size_t begin, size_t end = len>
  constexpr str_const<end - begin> substr() const noexcept
    requires(end > begin && end <= len)
  {
    str_const<end - begin> result;
    for (size_t i = 0; i != end - begin; ++i) {
      result.value[i] = value[i + begin];
    }
    return result;
  }
  constexpr char operator[](size_t i)
  {
    return value[i];
  };
  constexpr static size_t length = len;
};
template<size_t strlen>
str_const(char const (&lit)[strlen]) -> str_const<strlen>;

namespace string_literal {
template<str_const string>
consteval auto operator""_c()
{
  return string;
}
}  // namespace string_literal

}  // namespace util
