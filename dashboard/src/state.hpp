#include "gui.hpp"
#include "io.hpp"
#include <algorithm>
#include <asio/any_io_executor.hpp>
#include <asio/awaitable.hpp>
#include <asio/io_context.hpp>
#include <asio/strand.hpp>
#include <chrono>
#include <fmt/chrono.h>
#include <fmt/core.h>
#include <memory>
#include <utility>

using Strand =
  asio::strand<asio::io_context::basic_executor_type<std::allocator<void>, 0>>;

struct PID
{
  float p = 1.0, i = 0.1, d = 0.0;
};

// Unsafe in multithreaded context, needs strand for real synchronization
struct Dataset
{
  using Duration = std::chrono::duration<double>;
  using Timestamp =
    std::chrono::time_point<std::chrono::high_resolution_clock, Duration>;
  struct Datapoint
  {
    double data;
    double time;
  };

  Dataset()
  {
    data.resize(10000, {});
  }

  void push_front(double f)
  {
    Timestamp now = std::chrono::high_resolution_clock::now();
    data.at(newpos) = Datapoint{ f, now.time_since_epoch().count() };
    newpos += 1;
    if (newpos >= data.size())
      newpos = 0;
  }

  void resize(size_t size)
  {
    data.resize(size);
  }

  size_t offset() const noexcept
  {
    return newpos;
  }

  std::span<Datapoint const> get_datapoints() const
  {
    return data;
  }

  static std::pair<double, double> get_timespan(Duration d)
  {
    Timestamp now = std::chrono::high_resolution_clock::now();
    Timestamp before = now - d;
    return { before.time_since_epoch().count(),
             now.time_since_epoch().count() };
  }

private:
  std::vector<Datapoint> data;
  size_t newpos = 0;
};

struct State
{
  State(asio::io_context&);
  ~State() = default;
  void xscale(size_t max);
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
