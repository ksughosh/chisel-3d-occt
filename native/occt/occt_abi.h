// SPDX-License-Identifier: LGPL-2.1-only
// ABI contract of libchisel_occt.so (spec 16 8.2a). kChiselOcctAbi is what chisel_occt_abi() returns and what the host's
// kernel_loader checks after dlopen(). kPublicHeadersFnv1a64 is the FNV-1a 64 hash over the bytes of the public headers, in this
// order: chisel/kernel/geometry_kernel.h, chisel/kernel/kernel_types.h, chisel/common/uid.h. native/core/tests/kernel_abi_guard_test.cpp
// fails when the headers change without this file being updated: bump kChiselOcctAbi (the library and the host must agree) and
// paste the new hash that the test prints.
#pragma once
#include <cstdint>

namespace chisel {
constexpr int kChiselOcctAbi = 1;
constexpr uint64_t kPublicHeadersFnv1a64 = 0xcf75d375c7c488a6ull;
}  // namespace chisel
