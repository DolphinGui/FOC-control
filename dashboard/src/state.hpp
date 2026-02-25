#include "gui.hpp"
#include <algorithm>
#include <asio/any_io_executor.hpp>
#include <asio/awaitable.hpp>
#include <asio/io_context.hpp>
#include <asio/strand.hpp>
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

  void push_front(float f)
  {
    data.back() = f;
    std::rotate(data.begin(), data.end() - 1, data.end());
  }
};

struct State
{
  State(asio::io_context&);
  ~State() = default;
  void xscale(size_t max);
  void yscale(std::string_view, float max);
  void erase_set(std::string_view data);
  void insert_data(std::string_view set, float);

  asio::awaitable<std::vector<std::pair<std::string_view, Dataset const *>>> list_data() const;

private:
  struct Inner;
  std::shared_ptr<Inner> inner_state;
  Strand strand;
};
