#pragma once

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
std::span<char> encode(std::span<uint8_t, size> input,
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
template<std::regular... Args>
constexpr auto encode_message(Args... args)
  -> std::pair<std::array<char, 4 * ((sizeof(Args) + ...) + 2) / 3 + 1>, size_t>
{
  std::array<uint8_t, (sizeof(Args) + ...)> data;
  size_t offset = 0;
  ((std::memcpy(
      data.data() + offset, reinterpret_cast<uint8_t*>(&args), sizeof(args)),
    offset += sizeof(args)),
   ...);
  std::array<char, 4 * (data.size() + 2) / 3 + 1> result;
  size_t len = encode(std::span(data), std::span(result)).size();
  return { result, len };
}

}  // namespace b64