<!-- SPDX-License-Identifier: LGPL-2.1-only -->
# OCCT L1 build (ADR-001)

`tools/occt/build_occt.sh android|android-static|host` clones OCCT `V8_0_1` (commit `b8f597c677811d1f9f4d8a97f5ae2825c0353a42`, verified after clone) and builds shared libraries.

Options used (all accepted by V8_0_1 without renaming; no reconciliation was needed after `cmake -LH`):
`-DBUILD_LIBRARY_TYPE=Shared -DBUILD_MODULE_FoundationClasses=ON -DBUILD_MODULE_ModelingData=ON -DBUILD_MODULE_ModelingAlgorithms=ON`
`-DBUILD_MODULE_Visualization=OFF -DBUILD_MODULE_ApplicationFramework=OFF -DBUILD_MODULE_DataExchange=OFF -DBUILD_MODULE_Draw=OFF -DBUILD_MODULE_DETools=OFF`
`-DUSE_FREETYPE=OFF -DUSE_TK=OFF -DUSE_OPENGL=OFF -DUSE_GLES2=OFF -DUSE_FREEIMAGE=OFF -DUSE_RAPIDJSON=OFF -DUSE_DRACO=OFF -DUSE_TBB=OFF -DUSE_VTK=OFF -DBUILD_DOC_Overview=OFF`
Android adds the NDK toolchain, `ANDROID_ABI=arm64-v8a`, `ANDROID_PLATFORM=android-30`, `ANDROID_STL=c++_shared` and `-Wl,-z,max-page-size=16384 -Wl,-z,common-page-size=16384`.

Linked toolkits (13): TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool TKShHealing TKMesh (their NEEDED closure is inside this set).
Use: host tests `CHISEL_CMAKE_ARGS=-DCHISEL_KERNEL=occt CHISEL_BUILD_DIR=build/host-occt tools/host_test.sh`; Android `./gradlew assembleDebug -Pchisel.kernel=occt` (a Gradle `Copy` task stages the 13 libs into a generated jniLibs dir).
LGPL 2.1 with exception: libraries ship unmodified as shared objects; source URL and commit are in `app/src/main/assets/licenses.json`.

## android-static (KERN-0, ADR-019)
`tools/occt/build_occt.sh android-static` builds the same trimmed module set as static archives (arm64-v8a, android-30, c++_shared) into `third_party/occt/android-arm64-static/lib/*.a` (gitignored) with `-O2 -ffunction-sections -fdata-sections -fvisibility=hidden -fvisibility-inlines-hidden`. Env: `OCCT_STATIC_OPT=-Os`, `OCCT_STATIC_LTO=thin`, `OCCT_STATIC_TAG=<suffix>` (separate build/install dir) for size experiments.
`native/occt/CMakeLists.txt` links 13 archives plus the OCCT adapter into ONE `libchisel_occt.so` (`--gc-sections --icf=all`, `exports.map` = `chisel_occt_abi`, `chisel_occt_make`, 16 KB). Build: `cmake -S native -B build/kern0-android -G Ninja -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-30 -DANDROID_STL=c++_shared -DCMAKE_BUILD_TYPE=Release -DCHISEL_KERNEL=occt && cmake --build build/kern0-android --target chisel_occt`. Check: `tools/occt/check_occt_lib.sh build/kern0-android/occt/libchisel_occt.so` (stripped size, LOAD align, NEEDED, exports). Host mode is unchanged (shared).
