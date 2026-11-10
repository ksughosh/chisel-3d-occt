// SPDX-License-Identifier: LGPL-2.1-only OR LicenseRef-Chisel-Project
#include "chisel/kernel/geometry_kernel.h"
#include <cstdio>
#include <string>

namespace chisel {

Uid Uid::fromHex(const std::string& hex) {
  Uid u;
  u.id = std::stoull(hex, nullptr, 16);
  return u;
}

std::string Uid::hex() const {
  char buf[17];
  snprintf(buf, sizeof(buf), "%016llx", (unsigned long long)id);
  return std::string(buf);
}

}  // namespace chisel
