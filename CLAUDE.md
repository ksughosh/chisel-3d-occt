# chisel-3d-occt: Agent Guidelines

This is the **public, LGPL-2.1 mirror** of the Chisel 3D OCCT geometry-kernel adapter. It exists so the LGPL-covered code that links OCCT is publicly available (Chisel 3D decision U26, spec 16 section 8.2a). The Chisel 3D app itself is closed source and lives in a separate private repository.

## 1. How this repo is used
- It is mounted in the private app repo as the git submodule `third_party/occt-public`.
- **It is a mirror, not the build input.** The authoritative sources are the same paths in the private repo. Change them there first, then mirror here in one commit that names the source commit.
- Do not edit files here from inside the private repo's working tree without coordinator approval, and never push without it.

## 2. Current state (2026-11-11)
- Byte-identical mirror of the private-tree publish set, same relative paths (list in `MIRRORED_FILES.txt`, source commit in `MIRROR_SOURCE_COMMIT`): the real adapter `native/core/src/kernel/occt_kernel.cpp`, `kernel_common.cpp`, `uid.cpp`, the three public headers, `native/occt/{occt_entry.cpp,occt_abi.h,exports.map,CMakeLists.txt}`, `tools/occt/*`, `LICENSES/`.
- To update: copy the listed files from the private tree, check with `cmp`, rewrite `MIRROR_SOURCE_COMMIT`, commit naming that commit.
- Open: CI equality check in the private repo; Android build of `libchisel_occt.so` + spec 16 8.2a readelf/nm checks not yet run; legal review L1-L5 pending (private runbook U30).

## 3. Rules
- Licence: `SPDX-License-Identifier: LGPL-2.1-only` on every file. Keep `LICENSE` as the unmodified FSF LGPL-2.1 text.
- No OCCT type may appear in `include/chisel/kernel/geometry_kernel.h`; OCCT headers only in `src/kernel/`.
- No closed-source Chisel code (UI, Kotlin, renderer, document model) goes in this repo.
- Android builds: NDK r28, arm64-v8a, `c++_shared`, link with `-Wl,-z,max-page-size=16384` (16 KB pages are mandatory).
- User-facing strings never say "OCCT" (that rule applies to the app; this repo has no UI).
