# chisel-3d-occt

Source of the LGPL-covered part of Chisel 3D, a pen-and-touch CAD app for Android tablets: the
adapter that links the Open CASCADE Technology (OCCT) geometry kernel, and the scripts that build it.

## Licence

Every file here is **LGPL-2.1-only** (`SPDX-License-Identifier` in each file; full text in `LICENSE` and
`LICENSES/LGPL-2.1.txt`). OCCT itself is LGPL-2.1 with the OCCT exception 1.0 (`LICENSES/OCCT-LGPL-exception-1.0.txt`).
The interface headers (`geometry_kernel.h`, `kernel_types.h`, `uid.h`) are dual-licensed `LGPL-2.1-only OR
LicenseRef-Chisel-Project`. The rest of Chisel 3D (UI, document model, renderer) is a separate, closed-source work that
talks to this library only through the two exported C functions in `native/occt/exports.map`.

## What is here

Files keep the same relative paths as in the Chisel 3D source tree (list: `MIRRORED_FILES.txt`; the Chisel 3D commit
they were copied from: `MIRROR_SOURCE_COMMIT`).

| Path | Role |
|---|---|
| `native/occt/occt_entry.cpp`, `occt_abi.h`, `exports.map`, `CMakeLists.txt` | `libchisel_occt.so`: entry points `chisel_occt_abi` / `chisel_occt_make`, ABI version, version script, build |
| `native/core/src/kernel/occt_kernel.cpp` | The OCCT adapter (implements `IGeometryKernel`) |
| `native/core/src/kernel/kernel_common.cpp`, `native/core/src/common/uid.cpp` | Small helpers the adapter needs |
| `native/core/include/chisel/kernel/geometry_kernel.h`, `kernel_types.h`, `native/core/include/chisel/common/uid.h` | Interface between the library and the app |
| `tools/occt/build_occt.sh`, `README.md`, `check_occt_lib.sh` | Build OCCT 8.0.1 from upstream source and check the result |

## Building `libchisel_occt.so`

1. Build the OCCT static archives: `tools/occt/build_occt.sh android-static` (Android NDK r28, arm64-v8a; OCCT source is
   fetched from the upstream tag, see `tools/occt/README.md`).
2. Configure `native/occt` with the NDK CMake toolchain (`-DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-30
   -DANDROID_STL=c++_shared -DCHISEL_THIRD_PARTY=<dir with occt/android-arm64-static and src/glm>`) and build target
   `chisel_occt`. The link uses 16 KB pages (`-z max-page-size=16384`), required on Android 15+.
3. `tools/occt/check_occt_lib.sh` verifies exports and alignment.

## Replacing the library in the app

Replace `lib/arm64-v8a/libchisel_occt.so` in the Chisel 3D APK with your build, run `zipalign -P 16`, re-sign with
`apksigner`, and reinstall. Your build must report the same ABI version (`kChiselOcctAbi` in `native/occt/occt_abi.h`).

## Status

- The Chisel 3D release build currently ships the basic (mesh) engine and no OCCT. Shipping `libchisel_occt.so` and
  loading it at run time is planned (decision ADR-019); this repository already holds its exact sources.
- This repository is a mirror: changes are made in the Chisel 3D tree first and copied here byte-for-byte.
