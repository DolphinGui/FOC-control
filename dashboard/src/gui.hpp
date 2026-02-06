#pragma once

#include <memory>
#include <vector>

struct State
{
  bool show_demo;
  std::vector<float> data;
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
