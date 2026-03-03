#include "coro.hpp"

#include <pico/error.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <string_view>

template<typename Byte = std::uint8_t>
struct async_read_bin final : co::Task<std::span<Byte const>>
{
  co::Microseconds poll() override
  {
    using namespace std::chrono_literals;
    if (buffer.end() == write_ptr)
      return 0us;
    for (int ch = stdio_getchar_timeout_us(0); write_ptr < buffer.end();
         ch = stdio_getchar_timeout_us(0)) {
      if (ch == PICO_ERROR_TIMEOUT)
        break;
      *write_ptr++ = ch;
    }
    return std::distance(write_ptr, buffer.end()) * 100ms;
  }

  std::span<Byte const> value() override
  {
    return buffer;
  }

  async_read_bin() = default;
  async_read_bin(std::span<Byte> buf)
    : buffer(buf)
    , write_ptr(buf.begin())
  {
  }
  async_read_bin(async_read_bin const&) = delete;
  async_read_bin(async_read_bin&& r)
    : buffer(r.buffer)
    , write_ptr(r.write_ptr)
  {
    r.buffer = {};
    r.write_ptr = {};
  }
  ~async_read_bin() = default;

private:
  std::span<Byte> buffer{};
  std::span<char>::iterator write_ptr;
};

struct async_read_str final : co::Task<std::string_view>
{
  co::Microseconds poll() override
  {
    using namespace std::chrono_literals;
    if (done || write_len >= buffer.size())
      return 0us;
    for (int ch = stdio_getchar_timeout_us(0); write_len < buffer.size();
         ch = stdio_getchar_timeout_us(0)) {
      if (ch == PICO_ERROR_TIMEOUT)
        break;
      if (ch == terminator) {
        done = true;
        break;
      }
      buffer[write_len] = ch;
      write_len += 1;
    }
    return (buffer.size() - write_len) * 100ms;
  }

  std::string_view value() override
  {
    size_t len = 0;
    for (; len < buffer.size(); ++len) {
      if (buffer[len] == '\0')
        break;
    }
    return std::string_view(buffer.data(), len);
  }

  async_read_str() = default;
  async_read_str(std::span<char> buf, char terminate = '\n')
    : buffer(buf)
    , terminator(terminate)
  {
  }
  async_read_str(async_read_str const&) = delete;
  async_read_str(async_read_str&& r)
    : buffer(r.buffer)
  {
    r.buffer = {};
  }
  ~async_read_str() = default;

private:
  std::span<char> buffer{};
  size_t write_len = 0;
  char terminator{};
  bool done = false;
};
