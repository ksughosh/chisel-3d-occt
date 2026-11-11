// SPDX-License-Identifier: LGPL-2.1-only
// The only exported surface of libchisel_occt.so (spec 16 6.1). KERN-2's kernel_loader dlopen()s the library, checks
// chisel_occt_abi() == kChiselOcctAbi (occt_abi.h) and calls chisel_occt_make(); ownership of the returned kernel passes to the caller, which must
// delete it through IGeometryKernel's virtual destructor (same C++ runtime: both libs use libc++_shared.so).
#include "chisel/kernel/geometry_kernel.h"
#include "occt_abi.h"

namespace chisel { std::unique_ptr<IGeometryKernel> makeOcctKernel(); }

extern "C" {
__attribute__((visibility("default"))) int chisel_occt_abi() { return chisel::kChiselOcctAbi; }
__attribute__((visibility("default"))) chisel::IGeometryKernel* chisel_occt_make() { return chisel::makeOcctKernel().release(); }
}
