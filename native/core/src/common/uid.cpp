// SPDX-License-Identifier: LGPL-2.1-only
#include "chisel/common/uid.h"
#include <cstdio>
#include <mutex>
#include <random>

namespace chisel {

Uid Uid::random() {
  static std::mutex m;
  static std::mt19937_64 rng{std::random_device{}()};
  std::lock_guard<std::mutex> lk(m);
  Uid u;
  do { u.hi = rng(); u.lo = rng(); } while (u.isNil());
  return u;
}

std::string Uid::hex() const {
  char buf[33];
  std::snprintf(buf, sizeof buf, "%016llx%016llx", static_cast<unsigned long long>(hi), static_cast<unsigned long long>(lo));
  return buf;
}

Uid Uid::fromHex(const std::string& s) {
  if (s.size() != 32) return {};
  Uid u;
  for (int i = 0; i < 32; ++i) {
    char c = s[i]; int v;
    if (c >= '0' && c <= '9') v = c - '0';
    else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
    else return {};
    uint64_t& part = i < 16 ? u.hi : u.lo;
    part = (part << 4) | static_cast<uint64_t>(v);
  }
  return u;
}

}  // namespace chisel
