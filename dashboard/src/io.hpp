#pragma once

#include <asio/io_context.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct SerialInfo
{
  std::string name, path;
};

std::vector<SerialInfo> list_serial_devices();

using Callback = std::move_only_function<void(std::string_view type, double)>;

// Cannot be deleted
struct DeviceManager
{
  DeviceManager();
  DeviceManager(DeviceManager&&);
  ~DeviceManager();

  void connect(asio::io_context&, SerialInfo const&, Callback callback);
  void disconnect();

private:
  struct Inner;
  std::unique_ptr<Inner> inner;
};
