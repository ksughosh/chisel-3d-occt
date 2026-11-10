#!/usr/bin/env bash
# SPDX-License-Identifier: LGPL-2.1-only OR LicenseRef-Chisel-Project
# Build libchisel_occt.so for Android ARM64.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

ANDROID_NDK="${ANDROID_NDK:-$HOME/Library/Android/sdk/ndk/28.0.*.*}"
ANDROID_ABI="arm64-v8a"
ANDROID_API="30"
BUILD_TYPE="Release"
BUILD_DIR="build/android-arm64"

if [[ ! -d "$ANDROID_NDK" ]]; then
  echo "ERROR: Android NDK not found at $ANDROID_NDK"; exit 1
fi
if [[ -z "${OCCT_ROOT:-}" ]]; then
  echo "ERROR: OCCT_ROOT not set. export OCCT_ROOT=/path/to/occt/android-arm64"; exit 1
fi

cmake -S . -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_TOOLCHAIN_FILE="$ANDROID_NDK/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI="$ANDROID_ABI" \
  -DANDROID_PLATFORM="android-$ANDROID_API" \
  -DANDROID_STL="c++_shared" \
  -DCHISEL_KERNEL=occt \
  -DCHISEL_HOST=OFF \
  -DOCCT_ROOT="$OCCT_ROOT" \
  -DCMAKE_FIND_ROOT_PATH="$OCCT_ROOT" \
  -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ON \
  -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ON \
  -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ON

cmake --build "$BUILD_DIR" --config "$BUILD_TYPE" -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc)"
echo "Build complete: $BUILD_DIR/libchisel_occt.so"
