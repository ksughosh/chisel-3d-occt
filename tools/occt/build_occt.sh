#!/usr/bin/env bash
# SPDX-License-Identifier: LGPL-2.1-only
# OCCT L1 build (ADR-001, ADR-019). Usage: tools/occt/build_occt.sh <android|android-static|host>
# android-static (KERN-0): static archives for native/occt (one pruned libchisel_occt.so). Env overrides (size experiments):
#   OCCT_STATIC_OPT=-O2(default)|-Os   OCCT_STATIC_LTO=none|thin   OCCT_STATIC_TAG=<suffix>  (suffix is appended to build dir and install dir)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; cd "$ROOT"
OCCT_TAG=V8_0_1; OCCT_COMMIT=b8f597c677811d1f9f4d8a97f5ae2825c0353a42
SDK="${ANDROID_HOME:-$HOME/Library/Android/sdk}"; CM="$SDK/cmake/3.31.6/bin"; NDK="$SDK/ndk/28.2.13676358"
[[ -d third_party/src/occt ]] || git clone --depth 1 --branch "$OCCT_TAG" https://github.com/Open-Cascade-SAS/OCCT.git third_party/src/occt
test "$(git -C third_party/src/occt rev-parse HEAD)" = "$OCCT_COMMIT" || { echo "FAIL: OCCT commit mismatch"; exit 1; }
COMMON=(-G Ninja -DCMAKE_MAKE_PROGRAM="$CM/ninja" -DCMAKE_BUILD_TYPE=Release -DBUILD_LIBRARY_TYPE=Shared
  -DBUILD_MODULE_FoundationClasses=ON -DBUILD_MODULE_ModelingData=ON -DBUILD_MODULE_ModelingAlgorithms=ON
  -DBUILD_MODULE_Visualization=OFF -DBUILD_MODULE_ApplicationFramework=OFF -DBUILD_MODULE_DataExchange=OFF -DBUILD_MODULE_Draw=OFF -DBUILD_MODULE_DETools=OFF
  -DUSE_FREETYPE=OFF -DUSE_TK=OFF -DUSE_OPENGL=OFF -DUSE_GLES2=OFF -DUSE_FREEIMAGE=OFF -DUSE_RAPIDJSON=OFF -DUSE_DRACO=OFF -DUSE_TBB=OFF -DUSE_VTK=OFF -DBUILD_DOC_Overview=OFF)
case "${1:?android|android-static|host}" in
  android)
    "$CM/cmake" -S third_party/src/occt -B build/occt-android "${COMMON[@]}" -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" -DANDROID_ABI=arm64-v8a \
      -DANDROID_PLATFORM=android-30 -DANDROID_STL=c++_shared -DINSTALL_DIR="$PWD/third_party/occt/android-arm64" \
      -DCMAKE_SHARED_LINKER_FLAGS="-Wl,-z,max-page-size=16384 -Wl,-z,common-page-size=16384"
    "$CM/cmake" --build build/occt-android -j"$(sysctl -n hw.ncpu)" && "$CM/cmake" --install build/occt-android ;;
  android-static)
    OPT="${OCCT_STATIC_OPT:--O2}"; LTO="${OCCT_STATIC_LTO:-none}"; TAG="${OCCT_STATIC_TAG:-}"
    SF="$OPT -DNDEBUG -ffunction-sections -fdata-sections -fvisibility=hidden -fvisibility-inlines-hidden"
    [[ "$LTO" == thin ]] && SF="$SF -flto=thin"
    BD="build/occt-android-static$TAG"; ID="$PWD/third_party/occt/android-arm64-static$TAG"
    STATIC=("${COMMON[@]/-DBUILD_LIBRARY_TYPE=Shared/-DBUILD_LIBRARY_TYPE=Static}")
    "$CM/cmake" -S third_party/src/occt -B "$BD" "${STATIC[@]}" -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" -DANDROID_ABI=arm64-v8a \
      -DANDROID_PLATFORM=android-30 -DANDROID_STL=c++_shared -DINSTALL_DIR="$ID" \
      -DCMAKE_C_FLAGS_RELEASE="$SF" -DCMAKE_CXX_FLAGS_RELEASE="$SF"
    "$CM/cmake" --build "$BD" -j"$(sysctl -n hw.ncpu)" && "$CM/cmake" --install "$BD"
    echo "android-static OK: $ID/lib ($(ls "$ID"/lib/*.a | wc -l | tr -d ' ') archives; opt=$OPT lto=$LTO)" ;;
  host)
    "$CM/cmake" -S third_party/src/occt -B build/occt-host "${COMMON[@]}" -DINSTALL_DIR="$PWD/third_party/occt/macos-arm64"
    "$CM/cmake" --build build/occt-host -j"$(sysctl -n hw.ncpu)" && "$CM/cmake" --install build/occt-host ;;
esac
