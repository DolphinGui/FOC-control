#include "io.hpp"

#include <asio/awaitable.hpp>
#include <asio/bind_cancellation_slot.hpp>
#include <asio/buffer.hpp>
#include <asio/co_spawn.hpp>
#include <asio/completion_condition.hpp>
#include <asio/defer.hpp>
#include <asio/detached.hpp>
#include <asio/post.hpp>
#include <asio/read_until.hpp>
#include <asio/serial_port.hpp>
#include <asio/steady_timer.hpp>
#include <asio/streambuf.hpp>
#include <asio/use_awaitable.hpp>
#include <asio/use_future.hpp>
#include <asio/write.hpp>
#include <cstring>
#include <filesystem>
#include <fmt/base.h>
#include <fmt/ranges.h>
#include <iostream>
#include <stdexcept>

#include "base64.hpp"
/*
 Code is from https://github.com/nkinar/GetComPortList
 Only applies to the list_serial_devices() function
MIT License

Copyright (c) 2022 Nicholas J. Kinar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 */

static inline auto operator<=>(SerialInfo const& lhs, SerialInfo const& rhs)
{
  return lhs.name <=> rhs.name;
}

std::vector<SerialInfo> list_serial_devices()
{
  std::vector<SerialInfo> port_list;
#if defined(_WIN32) || defined(_WIN64)
  uint32_t const CHAR_NUM = 1024;
  uint32_t const MAX_PORTS = 255;
  std::string const COM_STR = "COM";
  char path[CHAR_NUM];
  for (uint32_t k = 0; k < MAX_PORTS; k++) {
    std::string port_name = COM_STR + std::to_string(k);
    DWORD test = QueryDosDevice(port_name.c_str(), path, CHAR_NUM);
    if (test == 0)
      continue;
    port_list.push_back(port_name);
  }
#elif defined(__linux__)
  namespace fs = std::filesystem;
  std::string const DEV_PATH = "/dev/serial/by-id";
  try {
    fs::path p(DEV_PATH);
    if (!fs::exists(DEV_PATH))
      return port_list;
    for (fs::directory_entry de : fs::directory_iterator(p)) {
      if (fs::is_symlink(de.symlink_status())) {
        fs::path symlink_points_at = fs::read_symlink(de);
        port_list.push_back(
          { de.path().filename(),
            std::string("/dev/") + symlink_points_at.filename().c_str() });
      }
    }
  } catch (fs::filesystem_error const& ex) {
  }
#elif defined(__APPLE__)
  namespace fs = std::filesystem;
  std::string const DEV_PATH = "/dev";
  std::regex const base_regex(R"(\/dev\/(tty|cu)\..*)");
  try {
    fs::path p(DEV_PATH);
    if (!fs::exists(DEV_PATH))
      return port_list;
    for (fs::directory_entry de : fs::directory_iterator(p)) {
      fs::path canonical_path = fs::canonical(de);
      std::string name = canonical_path.generic_string();
      std::smatch res;
      std::regex_search(name, res, base_regex);
      if (res.empty())
        continue;
      port_list.push_back(canonical_path.generic_string());
    }
  } catch (fs::filesystem_error const& ex) {
  }
#else
#error "Unknown platform, cannot detect serial devices"
#endif
  std::sort(port_list.begin(), port_list.end());
  return port_list;
}
struct DeviceManager::Inner
{

  void read()
  {
    async_read_until(
      port,
      buf,
      '\n',
      [this](std::error_code const& ec, std::size_t bytes_transferred) {
        std::string line;
        std::istream is(&buf);
        line.reserve(bytes_transferred);
        if (!ec) {
          std::getline(is, line);
          auto d = b64::decode(line);
          if (std::memcmp(
                &b64::schema_magic, d.data(), sizeof(b64::schema_magic)) == 0) {
            read_schema(std::span(d).subspan(2));
          } else if (std::memcmp(&b64::data_magic,
                                 d.data(),
                                 sizeof(b64::data_magic)) == 0) {
            read_msg(std::span(d).subspan(2));
          }
          // if schema magic doesn't match, it's probably just an invalid
          // packet
        } else {
          std::cerr << "Could not read due to error " << ec << "\n";
        }
        read();
      });
  }

  void request_schema()
  {
    asio::async_write(
      port, asio::buffer(schema_request), [](std::error_code e, size_t len) {
        if (len < schema_request.size()) {
          std::cerr << "Failed to write whole request to device\n";
        }
        if (e) {
          std::cerr << "Error writing to device: " << e.message() << '\n';
        }
      });
  }

  void read_schema(std::span<uint8_t> data)
  {
    size_t i = 0;
    schema.clear();
    schema_size = 0;
    while (i < data.size()) {
      char type = data[i];
      uint8_t size = data[i + 1];
      size_t namelen =
        std::strlen(reinterpret_cast<char*>(data.data() + i + 2));
      if (i + namelen > data.size()) {
        std::cerr << "Could not read schema, string not null-terminated\n";
        return;
      }
      auto name =
        std::string(reinterpret_cast<char*>(data.data() + i + 2), namelen);
      schema.emplace_back(type, size, name);
      schema_size += size;
      i += 2 + namelen + 1;
    }
  }

  void read_msg(std::span<uint8_t> data)
  {
    if (schema.empty())
      return;
    if (data.size_bytes() != schema_size) {
      std::cerr << "Unknown message, size is wrong\n";
      return;
    }
    size_t i = 0;
    for (auto const& param : schema) {
      double result;
      switch (param.type) {
        case b64::runtime_param::integer:
          // because of little endian, we can just use 1 case for
          // everything it it works out
          {
            uint64_t number = 0;
            memcpy(&number, data.data() + i, param.size);
            result = number;
          }
          break;
        case b64::runtime_param::floating:
          if (param.size == 4) {
            float f = 0;
            memcpy(&f, data.data() + i, param.size);
            result = f;
          } else if (param.size == 8) {
            double f = 0;
            memcpy(&f, data.data() + i, param.size);
            result = f;
          } else {
            std::cerr << "Unsupported floating point size in schema: "
                      << param.size << '\n';

            return;
          }
          break;
        default:
          std::cerr << "Unknown schema type " << param.type << '\n';
          return;
      }
      cb(param.name, result);
      i += param.size;
    }
  }

  asio::io_context* io;
  asio::serial_port port;
  Callback cb;
  asio::streambuf buf;
  std::vector<b64::runtime_param> schema;
  size_t schema_size = 0;
  static constexpr std::string_view schema_request = "getschema\n";
};

DeviceManager::DeviceManager() = default;
DeviceManager::DeviceManager(DeviceManager&&) = default;
DeviceManager::~DeviceManager() = default;

void DeviceManager::disconnect()
{
  inner.reset();
}

void DeviceManager::connect(asio::io_context& io,
                            SerialInfo const& info,
                            Callback callback)

{
  using namespace asio;
  disconnect();
  inner = std::make_unique<Inner>(
    &io, serial_port(io, info.path), std::move(callback));
  inner->request_schema();
  inner->read();
}
