#include "gui.hpp"
#include "io.hpp"
#include "state.hpp"
#include <asio/any_io_executor.hpp>
#include <asio/bind_cancellation_slot.hpp>
#include <asio/cancellation_signal.hpp>
#include <asio/cancellation_type.hpp>
#include <asio/co_spawn.hpp>
#include <asio/detached.hpp>
#include <asio/io_context.hpp>
#include <asio/signal_set.hpp>
#include <asio/steady_timer.hpp>
#include <asio/use_awaitable.hpp>
#include <fmt/base.h>
#include <fmt/ranges.h>
#include <stdexcept>

asio::awaitable<void> poll(asio::io_context& io, GUI& g, State& s)
{
  using namespace asio;
  while (co_await g.poll(s)) {
    steady_timer t(io, chrono::milliseconds(10));
    co_await t.async_wait();
  }
  io.stop();
}

int main()
{
  try {
    asio::io_context context;
    State s(context);
    GUI g;
    using namespace std::chrono_literals;
    asio::co_spawn(context, poll(context, g, s), asio::detached);

    asio::signal_set signals(context, SIGINT, SIGTERM);
    signals.async_wait([&](auto, auto) { context.stop(); });
    context.run();
  } catch (std::runtime_error const& r) {
    fmt::println("Encountered error: {}", r.what());
  }
}
