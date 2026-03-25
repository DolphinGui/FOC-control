#pragma once

#include "conststr.hpp"
#include <bit>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <vector>

namespace b64 {
constexpr std::array<char const, 65> alphabet = {
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
};

constexpr std::array<uint8_t, 256> reverse = [] {
  std::array<uint8_t, 256> a{};
  a.fill(0xff);
  for (uint8_t i = 0; i < 64; ++i)
    a[alphabet[i]] = i;
  return a;
}();

constexpr std::vector<uint8_t> decode(std::span<char const> input)
{
  std::vector<uint8_t> result;
  result.reserve(input.size() / 4 * 3);
  if (input.size() % 4 != 0)
    throw std::runtime_error("invalid base64 length");

  for (size_t i = 0; i < input.size(); i += 4) {
    char c0 = input[i];
    char c1 = input[i + 1];
    char c2 = input[i + 2];
    char c3 = input[i + 3];

    uint8_t b0 = reverse[c0];
    uint8_t b1 = reverse[c1];

    if (b0 == 255 || b1 == 255)
      throw std::runtime_error("invalid base64 character");

    uint32_t chunk = (b0 << 18) | (b1 << 12);

    if (c2 == '=') {
      result.push_back((chunk >> 16) & 0xFF);
      break;
    }

    uint8_t b2 = reverse[static_cast<uint8_t>(c2)];
    if (b2 == 255)
      throw std::runtime_error("invalid base64 character");

    chunk |= (b2 << 6);

    if (c3 == '=') {
      result.push_back((chunk >> 16) & 0xFF);
      result.push_back((chunk >> 8) & 0xFF);
      break;
    }

    uint8_t b3 = reverse[c3];
    if (b3 == 255)
      throw std::runtime_error("invalid base64 character");

    chunk |= b3;

    result.push_back((chunk >> 16) & 0xFF);
    result.push_back((chunk >> 8) & 0xFF);
    result.push_back(chunk & 0xFF);
  }

  return result;
}

template<size_t size>
constexpr std::span<char> encode(std::span<uint8_t, size> input,
                                 std::span<char, 4 * (size + 2) / 3 + 1> output,
                                 char terminator = '\n')
  requires(size != std::dynamic_extent)
{
  size_t in_i = 0;
  size_t out_i = 0;

  while (in_i + 3 <= input.size()) {
    uint32_t chunk = (uint32_t(input[in_i]) << 16) |
                     (uint32_t(input[in_i + 1]) << 8) |
                     (uint32_t(input[in_i + 2]));

    output[out_i++] = alphabet[(chunk >> 18) & 0x3F];
    output[out_i++] = alphabet[(chunk >> 12) & 0x3F];
    output[out_i++] = alphabet[(chunk >> 6) & 0x3F];
    output[out_i++] = alphabet[chunk & 0x3F];
    in_i += 3;
  }

  size_t const remaining = input.size() - in_i;

  if (remaining == 1) {
    uint32_t chunk = uint32_t((input[in_i])) << 16;

    output[out_i++] = alphabet[(chunk >> 18) & 0x3F];
    output[out_i++] = alphabet[(chunk >> 12) & 0x3F];
    output[out_i++] = '=';
    output[out_i++] = '=';
  } else if (remaining == 2) {
    uint32_t chunk =
      (uint32_t((input[in_i])) << 16) | (uint32_t((input[in_i + 1])) << 8);
    output[out_i++] = alphabet[(chunk >> 18) & 0x3F];
    output[out_i++] = alphabet[(chunk >> 12) & 0x3F];
    output[out_i++] = alphabet[(chunk >> 6) & 0x3F];
    output[out_i++] = '=';
  }
  output[out_i++] = terminator;

  return output.subspan(0, out_i);
}

template<typename T>
constexpr void memcopy(uint8_t* dst, T src)
{
  auto a = std::bit_cast<std::array<uint8_t, sizeof(src)>>(src);
  for (size_t i = 0; i < sizeof(src); ++i) {
    dst[i] = a[i];
  }
}

template<typename... Args>
constexpr auto data_array(Args... args)
{
  std::array<uint8_t, (sizeof(Args) + ...)> data{};
  size_t offset = 0;
  ((memcopy(data.data() + offset, args), offset += sizeof(args)), ...);
  return data;
}

template<std::regular... Args>
constexpr auto encode_message(Args... args)
  -> std::pair<std::array<char, 4 * (4 + (sizeof(Args) + ...) + 2) / 3 + 1>,
               size_t>
{
  size_t magic = 0x7adada7a;
  std::array<uint8_t, 4 + (sizeof(Args) + ...)> data =
    data_array(magic, args...);
  std::array<char, 4 * (data.size() + 2) / 3 + 1> result;
  size_t len = encode(std::span(data), std::span(result)).size();
  return { result, len };
}

/*
 *
 */

template<typename>
struct typeparam;

template<std::floating_point F>
struct typeparam<F>
{
  constexpr static char type = 'f';
  constexpr static uint8_t size = sizeof(F);
};

template<std::integral I>
struct typeparam<I>
{
  constexpr static char type = 'i';
  constexpr static uint8_t size = sizeof(I);
};

template<typename p, auto name>
struct parameter
{
  using param = typeparam<p>;
  constexpr static auto serialized = data_array(param::type, param::size, name);
};

template<typename... Params>
constexpr auto create_schema()
{
  size_t magic = 0xdec0c0de;
  auto data = data_array(magic, Params::serialized..., '\n');
  std::array<char, 4 * (data.size() + 2) / 3 + 1> result{};
  size_t len = encode(std::span(data), std::span(result)).size();
  return std::pair{ result, len };
}

}  // namespace b64
