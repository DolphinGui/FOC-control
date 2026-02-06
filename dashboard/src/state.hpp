#include "broadcast.hpp"
#include "gui.hpp"
#include <asio/any_io_executor.hpp>
#include <functional>
#include <span>
#include <string>
#include <unordered_map>

struct PID
{
  float p = 1.0, i = 0.1, d = 0.0;
};

struct PostPIDMsg
{
  PID v, x;
};
struct GetPIDMsg
{
  PID v, x;
};

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

struct State
{
  State(Broadcaster&, asio::io_context&);
  ~State() = default;
  void xscale(size_t max);
  void yscale(std::string_view, float max);
  void new_dataset(std::string shortname, std::string longname);
  void delete_dataset(std::string_view data);

  std::vector<std::string_view> get_data_names();
  std::span<float> get_data(std::string_view data);
  std::string_view get_data_fullname(std::string_view data);
  float get_yscale(std::string_view);

  void poll();

  bool show_demo;
  bool alive;

private:
  struct Inner;
  std::shared_ptr<Inner> inner_state;
  PID v, x;
  struct Dataset;
  std::unordered_map<std::string, Dataset, string_hash, std::equal_to<>>
    datasets;
  Broadcaster::SharedListener observer;
  friend struct GUI;
  GUI gui;
};

struct State::Dataset
{
  float y_range = 1.0;
  std::string fullname;
  std::vector<float> data;

  Dataset(std::string&& name)
  {
    y_range = 1.0;
    fullname = std::move(name);
    data.resize(200, 0.0);
  }
};

struct RescaleXMsg
{
  size_t new_max;
};

struct AddDataMsg
{
  std::string set;
  float data;
};

struct AddSetMsg
{
  std::string shortname, longname;
};
