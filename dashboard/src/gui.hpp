#pragma once

#include <asio/any_io_executor.hpp>
#include <asio/awaitable.hpp>
#include <asio/io_context.hpp>
#include <asio/strand.hpp>
#include <memory>

struct State;

using Strand =
  asio::strand<asio::io_context::basic_executor_type<std::allocator<void>, 0>>;

struct GUI
{
  GUI();
  ~GUI();
  struct Internal;

  asio::awaitable<bool> poll(State&);

private:
  std::unique_ptr<Internal> inner;
};
