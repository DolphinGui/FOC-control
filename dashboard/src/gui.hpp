#pragma once

#include <asio/any_io_executor.hpp>
#include <asio/awaitable.hpp>
#include <memory>

struct State;

struct GUI
{
  GUI();
  ~GUI();
  struct Internal;

  void poll(State&);

private:
  std::unique_ptr<Internal> inner;
};

