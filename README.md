# chisel-3d-occt

OCCT (Open CASCADE Technology) bindings and build scripts for [Chisel 3D](https://github.com/ksughosh/chisel-3d), a pen-and-touch B-rep CAD application for Android tablets.

## Purpose

This public repository holds the OCCT-specific code that is subject to the **LGPL-2.1-or-later** license:

- OCCT build configuration for Android ARM64
- JNI/C++ kernel bindings (`occt_kernel.cpp` and related)
- Build scripts producing `libchisel_occt.so`

## License

OCCT bindings and build scripts in this repository are licensed under **[LGPL-2.1-or-later](LICENSE)**.

OCCT itself remains under its own license; this repository references OCCT source from its official distribution.

## Relationship to Chisel 3D

Chisel 3D (`ksughosh/chisel-3d`) references this repository as a dependency for the optional OCCT geometry kernel. The mesh kernel remains the default; OCCT is dynamically loaded (`dlopen`) after frame 1 with a mesh fallback. Per ADR-011, the literal string "OCCT" never appears in user-facing copy; the user-facing name is "precise geometry engine".
