#pragma once

#include <asio/any_io_executor.hpp>
#include <asio/awaitable.hpp>
#include <asio/buffer.hpp>
#include <asio/read.hpp>
#include <asio/serial_port.hpp>
#include <asio/use_awaitable.hpp>
#include <cstdint>
#include <stdexcept>

template<typename T, typename E = asio::any_io_executor>
using awaitable = asio::awaitable<T, E>;

using asio::use_awaitable;

struct Serial
{
  asio::serial_port se;
  inline awaitable<void> handle_serial()
  {
    std::array<uint8_t, 4> buf;
    auto l = co_await asio::async_read(se, asio::buffer(buf, 1), use_awaitable);
    if (l != 1)
      throw std::runtime_error("Failed to read command");
    switch (buf[0]) {
    }
    co_return;
  }

  void send_pid();
};
