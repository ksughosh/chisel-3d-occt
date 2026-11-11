# chisel-3d-occt: Agent Guidelines

This is the **public, LGPL-2.1 mirror** of the Chisel 3D OCCT geometry-kernel adapter. It exists so the LGPL-covered code that links OCCT is publicly available (Chisel 3D decision U26, spec 16 section 8.2a). The Chisel 3D app itself is closed source and lives in a separate private repository.

## 1. How this repo is used
- It is mounted in the private app repo as the git submodule `third_party/occt-public`.
- **It is a mirror, not the build input (yet).** The authoritative sources are in the private repo:
  - `native/core/src/kernel/occt_kernel.cpp` (adapter)
  - `native/core/include/chisel/kernel/geometry_kernel.h` (interface)
  - OCCT build files (`tools/occt/build_occt.sh`, `native/occt/CMakeLists.txt`, `exports.map`, `occt_entry.cpp`)
- Change those first, there; then mirror them here in one commit that names the source commit.
- Do not edit files here from inside the private repo's working tree without coordinator approval, and never push without it.

## 2. Current state (2026-11-11)
- Contents: `include/chisel/kernel/geometry_kernel.h` (interface), `src/kernel/occt_kernel.cpp` (**stub**: every method returns "not implemented"), `src/kernel/uid.cpp`, `CMakeLists.txt`, `build_android_arm64.sh`.
- Known gaps (architecture review H3): the real adapter is not published; the header lacks `BoolOp::Intersect` and `KernelCaps::intersect` (added in-tree by KERN-1/2); `occt_entry.cpp`, `exports.map` and the OCCT build script are missing; README says "LGPL-2.1-or-later" while source headers say `LGPL-2.1-only`; README describes a `dlopen` loader that does not exist yet.

## 3. Rules
- Licence: `SPDX-License-Identifier: LGPL-2.1-only` on every file. Keep `LICENSE` as the unmodified FSF LGPL-2.1 text.
- No OCCT type may appear in `include/chisel/kernel/geometry_kernel.h`; OCCT headers only in `src/kernel/`.
- No closed-source Chisel code (UI, Kotlin, renderer, document model) goes in this repo.
- Android builds: NDK r28, arm64-v8a, `c++_shared`, link with `-Wl,-z,max-page-size=16384` (16 KB pages are mandatory).
- User-facing strings never say "OCCT" (that rule applies to the app; this repo has no UI).
