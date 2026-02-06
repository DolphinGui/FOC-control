#include "broadcast.hpp"
#include "gui.hpp"
#include "state.hpp"
#include <asio/any_io_executor.hpp>
#include <asio/co_spawn.hpp>
#include <asio/detached.hpp>
#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>
#include <asio/use_awaitable.hpp>
#include <cmath>
#include <fmt/base.h>
#include <stdexcept>

asio::awaitable<void> emit_data(asio::any_io_executor io, Broadcaster& b)
{
  size_t i = 0;
  for (;;) {
    float angle = static_cast<float>(std::sin(i * 0.1));
    asio::steady_timer t(io, asio::chrono::milliseconds(10));
    co_await t.async_wait(asio::use_awaitable);
    b.emit(AddDataMsg{ "th", angle });
    i++;
  }
}

int main()
{
  try {
    asio::io_context context;
    Broadcaster broadcast;
    State s(broadcast, context);
    using namespace std::chrono_literals;
    broadcast.emit(AddSetMsg{ "th", "Angle" });
    asio::co_spawn(context.get_executor(),
                   emit_data(context.get_executor(), broadcast),
                   asio::detached);
    for (;;) {
      context.run();
    }
  } catch (std::runtime_error const& r) {
    fmt::println("Encountered error: {}", r.what());
  }
}
