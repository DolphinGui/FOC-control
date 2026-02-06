#include "broadcast.hpp"
#include "gui.hpp"
#include "state.hpp"
#include <algorithm>
#include <asio/co_spawn.hpp>
#include <asio/detached.hpp>
#include <asio/dispatch.hpp>
#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>
#include <asio/strand.hpp>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <vector>

asio::awaitable<void> poll_input(asio::any_io_executor io, State& s)
{
  while (s.alive) {
    asio::steady_timer t(io, asio::chrono::milliseconds(10));
    co_await t.async_wait(asio::use_awaitable);
    s.poll();
  }
}

State::State(Broadcaster& b, asio::io_context& e)
  : show_demo(true)
  , alive(true)
{
  using Listener = Broadcaster::Listener;
  auto strand = asio::make_strand(e.get_executor());
  this->gui.poll(*this);
  this->observer =
    std::make_shared<Listener>([this, strand](Broadcaster::MsgPtr m) {
      asio::dispatch(strand, [this, m] {
        if (auto* f = m->get_data<AddDataMsg>()) {
          auto n = this->datasets.find(f->set);
          if (n != this->datasets.end()) {
            auto& set = n->second.data;
            set.front() = f->data;
            std::rotate(set.begin(), set.begin() + 1, set.end());
          }
        } else if (auto* n = m->get_data<AddSetMsg>()) {
          this->new_dataset(n->shortname, n->longname);
        } else {
          return;
        }
        this->gui.poll(*this);
      });
    });
  b.subscribe(observer);
  asio::co_spawn(e, poll_input(strand, *this), asio::detached);
}

void State::poll()
{
  return this->gui.poll(*this);
}

void State::xscale(size_t size)
{
  for (auto&& [_, set] : this->datasets) {
    size_t old_size = set.data.size();
    set.data.resize(size, 0.0f);
    if (old_size < size) {
      size_t a = size - old_size;
      std::rotate(set.data.begin(), set.data.begin() + a, set.data.end());
    }
  }
}
void State::yscale(std::string_view, float max)
{
  for (auto&& [_, set] : this->datasets) {
    set.y_range = max;
  }
}
void State::new_dataset(std::string shortname, std::string longname)
{
  Dataset d(std::move(longname));
  datasets.emplace(std::make_pair(std::move(shortname), std::move(d)));
}
void State::delete_dataset(std::string_view key)
{
  auto a = datasets.find(key);
  if (a == datasets.end())
    throw std::out_of_range("Could not find dataset");
  datasets.erase(a);
}
std::span<float> State::get_data(std::string_view key)
{
  auto a = datasets.find(key);
  if (a == datasets.end())
    throw std::out_of_range("Could not find dataset");
  return a->second.data;
}
std::string_view State::get_data_fullname(std::string_view key)
{
  auto a = datasets.find(key);
  if (a == datasets.end())
    throw std::out_of_range("Could not find dataset");
  return a->second.fullname;
}

float State::get_yscale(std::string_view key)
{
  auto a = datasets.find(key);
  if (a == datasets.end())
    throw std::out_of_range("Could not find dataset");
  return a->second.y_range;
}

std::vector<std::string_view> State::get_data_names()
{
  auto get_name = [](auto i) {
    auto&& [_, d] = i;
    return std::string_view(d.fullname);
  };
  return datasets | std::views::transform(get_name) |
         std::ranges::to<std::vector<std::string_view>>();
}
