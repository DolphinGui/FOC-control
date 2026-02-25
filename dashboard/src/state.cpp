#include "state.hpp"
#include "gui.hpp"
#include <algorithm>
#include <asio/co_spawn.hpp>
#include <asio/detached.hpp>
#include <asio/dispatch.hpp>
#include <asio/io_context.hpp>
#include <asio/post.hpp>
#include <asio/steady_timer.hpp>
#include <asio/strand.hpp>
#include <asio/use_awaitable.hpp>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <vector>

struct string_hash
{
  using is_transparent = void;
  [[nodiscard]] size_t operator()(char const* txt) const
  {
    return std::hash<std::string_view>{}(txt);
  }
  [[nodiscard]] size_t operator()(std::string_view txt) const
  {
    return std::hash<std::string_view>{}(txt);
  }
  [[nodiscard]] size_t operator()(std::string const& txt) const
  {
    return std::hash<std::string>{}(txt);
  }
};

struct State::Inner
{
  Inner() = default;

  void xscale(size_t size)
  {
    for (auto&& [_, v] : datasets) {
      v.data.resize(size, 0.0);
    }
  }

  void yscale(std::string_view sv, float max)
  {
    auto d = this->datasets.find(sv);
    if (d != datasets.end()) {
      d->second.y_range = max;
    }
  }

  void erase_set(std::string_view set)
  {
    auto d = this->datasets.find(set);
    if (d != datasets.end()) {
      datasets.erase(d);
    }
  }

  void insert_data(std::string_view set, float f)
  {
    auto d = this->datasets.find(set);
    if (d != datasets.end()) {
      d->second.push_front(f);
    } else {
      auto [data, success] =
        this->datasets.emplace(std::string(set), Dataset{});
      if (!success)
        throw std::runtime_error("Could not insert data");
      data->second.push_front(f);
    }
  }

  std::vector<std::pair<std::string_view, Dataset const*>> list_data() const
  {
    std::vector<std::pair<std::string_view, Dataset const*>> results;
    results.reserve(datasets.size());
    for (auto&& [name, data] : datasets) {
      results.emplace_back(name, &data);
    }
    return results;
  }

private:
  std::unordered_map<std::string, Dataset, string_hash, std::equal_to<>>
    datasets;
};

State::State(asio::io_context& e)
  : inner_state(std::make_unique<Inner>())
  , strand(asio::make_strand(e))
{
}

void State::xscale(size_t max)
{
  asio::post(strand, [=, this] { this->inner_state->xscale(max); });
}
void State::yscale(std::string_view data, float max)
{
  asio::post(strand, [=, this] { this->inner_state->yscale(data, max); });
}
void State::erase_set(std::string_view data)
{
  asio::post(strand, [=, this] { this->inner_state->erase_set(data); });
}
void State::insert_data(std::string_view set, float data)
{
  asio::post(strand, [=, this] { this->inner_state->insert_data(set, data); });
}

asio::awaitable<std::vector<std::pair<std::string_view, Dataset const*>>>
State::list_data() const
{
  co_await asio::dispatch(strand, asio::use_awaitable);
  co_return this->inner_state->list_data();
}