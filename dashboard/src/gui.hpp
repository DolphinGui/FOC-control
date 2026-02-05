#pragma once

#include <memory>
struct GUI
{
  GUI();
  ~GUI();
  struct State;

  void poll();

private:
  std::unique_ptr<State> inner;
};
