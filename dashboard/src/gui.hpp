#pragma once

#include <memory>

struct State
{
  bool show_demo;
};

struct GUI
{
  GUI();
  ~GUI();
  struct Internal;

  void poll(State&);

private:
  std::unique_ptr<Internal> inner;
};
