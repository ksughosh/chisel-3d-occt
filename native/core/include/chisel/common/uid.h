// SPDX-License-Identifier: LGPL-2.1-only OR LicenseRef-Chisel-Project
#pragma once
#include <compare>
#include <cstdint>
#include <string>

namespace chisel {

struct Uid {
  uint64_t hi = 0, lo = 0;
  static Uid random();
  static Uid fromHex(const std::string& s);  // returns nil on malformed input
  std::string hex() const;                   // 32 lowercase hex chars
  bool isNil() const { return hi == 0 && lo == 0; }
  auto operator<=>(const Uid&) const = default;
};

}  // namespace chisel

namespace std {
template <> struct hash<chisel::Uid> {
  size_t operator()(const chisel::Uid& u) const noexcept { return static_cast<size_t>(u.hi * 0x9E3779B97F4A7C15ull ^ u.lo); }
};
}  // namespace std
