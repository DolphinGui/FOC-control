#include "gui.hpp"
#include "io.hpp"
#include <algorithm>
#include <asio/any_io_executor.hpp>
#include <asio/awaitable.hpp>
#include <asio/io_context.hpp>
#include <asio/strand.hpp>
#include <chrono>
#include <memory>

using Strand =
  asio::strand<asio::io_context::basic_executor_type<std::allocator<void>, 0>>;

struct PID
{
  float p = 1.0, i = 0.1, d = 0.0;
};

// Unsafe in multithreaded context, needs strand for real synchronization
struct Dataset
{
  std::vector<float> data;
  float y_range = 1.0;
  Dataset()
  {
    data.resize(200, 0.0);
  }

  void push_front(double f)
  {
    data.back() = f;
    std::rotate(data.begin(), data.end() - 1, data.end());
  }

  void resize(size_t size)
  {
    data.resize(size);
  }
};

struct State
{
  State(asio::io_context&);
  ~State() = default;
  void xscale(size_t max);
  void yscale(std::string_view, float max);
  void erase_set(std::string_view data);
  void insert_data(std::string_view set, double);
  void connect_device(SerialInfo const& info);

  asio::awaitable<std::vector<std::pair<std::string_view, Dataset const*>>>
  list_data() const;

private:
  struct Inner;
  std::shared_ptr<Inner> inner_state;
  Strand strand;
  DeviceManager device;
};
