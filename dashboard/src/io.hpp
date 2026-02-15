#pragma once

#include <string>
#include <vector>

struct SerialInfo
{
  std::string name, path;
};

std::vector<SerialInfo> list_serial_devices();
