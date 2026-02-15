#include "io.hpp"

#include <filesystem>
/*
 Code is from https://github.com/nkinar/GetComPortList
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
  const uint32_t CHAR_NUM = 1024;
  uint32_t const MAX_PORTS = 255;
  std::string const COM_STR = "COM";
  char path[CHAR_NUM];
  for (uint32_t k = 0; k < MAX_PORTS; k++) {
    std::string port_name = COM_STR + std::to_string(k);
    DWORD test = Query

      Code is from DosDevice(port_name.c_str(), path, CHAR_NUM);
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
