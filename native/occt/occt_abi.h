// SPDX-License-Identifier: LGPL-2.1-only
// ABI contract of libchisel_occt.so (spec 16 8.2a). kChiselOcctAbi is what chisel_occt_abi() returns and what the host's
// kernel_loader checks after dlopen(). kPublicHeadersFnv1a64 is the FNV-1a 64 hash over the bytes of the public headers, in this
// order: chisel/kernel/geometry_kernel.h, chisel/kernel/kernel_types.h, chisel/common/uid.h. native/core/tests/kernel_abi_guard_test.cpp
// fails when the headers change without this file being updated: bump kChiselOcctAbi (the library and the host must agree) and
// paste the new hash that the test prints.
#pragma once
#include <cstdint>

namespace chisel {
constexpr int kChiselOcctAbi = 3;   // 2: KERN-5 chamfer + draft AND the bend / STEP declarations in one public-header change (ADR-027, G3 5.3, G4 8.1c)
                                    // 3: C8-G5 transformBody + KernelCaps::transform (ADR-027 pattern, spec 17 G5 §3)
                                    // 3: C8-G5 transformBody + KernelCaps::transform; C8-G8 importStep + KernelCaps::stepImport (one bump, one push)
constexpr uint64_t kPublicHeadersFnv1a64 = 0xf40dff458c139a2eull;
}  // namespace chisel
