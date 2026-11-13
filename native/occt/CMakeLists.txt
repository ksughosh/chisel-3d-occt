# SPDX-License-Identifier: LGPL-2.1-only
# libchisel_occt.so (KERN-0, spec 16 6.1, ADR-019): OCCT static archives + the OCCT kernel adapter linked into ONE shared
# library, pruned by the linker (--gc-sections --icf=all) and exporting only chisel_occt_abi / chisel_occt_make.
# Archives come from `tools/occt/build_occt.sh android-static` (third_party/occt/android-arm64-static).
# Not packaged into the APK yet (KERN-1). Built only for the occt kernel (native/CMakeLists.txt).
cmake_minimum_required(VERSION 3.22)
if(NOT ANDROID)
  message(STATUS "chisel_occt: Android only (host keeps the shared OCCT build); skipped")
  return()
endif()
set(CHISEL_OCCT_ARCHIVE_DIR "${CHISEL_THIRD_PARTY}/occt/android-arm64-static" CACHE PATH "OCCT static archives root (build_occt.sh android-static)")
option(CHISEL_OCCT_REQUIRE_STATIC "Fail configure when the static archives are missing (KERN-1 turns this on)" OFF)
option(CHISEL_OCCT_IN_ALL "Build chisel_occt as part of 'all' (KERN-1 turns this on; until then only on request)" OFF)
set(CHISEL_OCCT_OPT "-O2" CACHE STRING "Optimisation flag for the adapter sources")
set(CHISEL_OCCT_LTO "none" CACHE STRING "none|thin: ThinLTO for the final link (archives must be built with the same OCCT_STATIC_LTO)")

if(NOT EXISTS "${CHISEL_OCCT_ARCHIVE_DIR}/lib/libTKernel.a")
  set(_msg "chisel_occt: OCCT static archives missing in ${CHISEL_OCCT_ARCHIVE_DIR}. Run tools/occt/build_occt.sh android-static")
  if(CHISEL_OCCT_REQUIRE_STATIC)
    message(FATAL_ERROR "${_msg}")
  endif()
  message(WARNING "${_msg} (target chisel_occt not created)")
  return()
endif()

# The 15 toolkits the adapter needs (KERN-5, ADR-027: + TKFillet for BRepFilletAPI_MakeChamfer, + TKOffset for BRepOffsetAPI_DraftAngle; TKFeat/TKHelix/TKHLR are not linked).
set(CHISEL_OCCT_TKS TKBO TKBool TKShHealing TKTopAlgo TKGeomAlgo TKGeomBase TKPrim TKMesh TKBRep TKG3d TKG2d TKMath TKernel TKFillet TKOffset TKDESTEP TKXSBase TKDE)   # KERN-6 (U38): STEP writer; add TKXCAF TKCAF TKLCAF TKCDF only if the link reports undefined XCAF symbols
set(_core "${CMAKE_CURRENT_SOURCE_DIR}/../core")
# LGPL publish set (KERN-2, spec 16 8.2a): exactly these seven translation units (KERN-5 added occt_bevel_guard/occt_chamfer/occt_draft). kernel_common.cpp holds the shared
# helpers (buildFaceRanges, NameMap::find, TopoName::str, loopSignedArea); uid.cpp holds Uid. No sketch, document or JSON code is compiled in.
add_library(chisel_occt SHARED
  occt_entry.cpp
  ${_core}/src/kernel/occt_kernel.cpp
  ${_core}/src/kernel/occt_bevel_guard.cpp
  ${_core}/src/kernel/occt_chamfer.cpp
  ${_core}/src/kernel/occt_draft.cpp
  ${_core}/src/kernel/occt_bend.cpp
  ${_core}/src/kernel/occt_step.cpp
  ${_core}/src/kernel/occt_step_import.cpp
  ${_core}/src/kernel/kernel_common.cpp
  ${_core}/src/common/uid.cpp)
if(NOT CHISEL_OCCT_IN_ALL)
  set_target_properties(chisel_occt PROPERTIES EXCLUDE_FROM_ALL ON)
endif()
target_include_directories(chisel_occt PRIVATE ${_core}/include)
target_include_directories(chisel_occt SYSTEM PRIVATE
  ${CHISEL_THIRD_PARTY}/src/eigen ${CHISEL_THIRD_PARTY}/src/glm ${CHISEL_THIRD_PARTY}/src/json ${CHISEL_THIRD_PARTY}/src/earcut/include
  ${CHISEL_OCCT_ARCHIVE_DIR}/include/opencascade)
target_compile_definitions(chisel_occt PRIVATE GLM_ENABLE_EXPERIMENTAL GLM_FORCE_RADIANS CHISEL_KERNEL_NAME="occt")
set_source_files_properties(${_core}/src/kernel/occt_kernel.cpp ${_core}/src/kernel/occt_bevel_guard.cpp ${_core}/src/kernel/occt_chamfer.cpp ${_core}/src/kernel/occt_draft.cpp ${_core}/src/kernel/occt_bend.cpp ${_core}/src/kernel/occt_step.cpp ${_core}/src/kernel/occt_step_import.cpp PROPERTIES COMPILE_OPTIONS "-Wno-deprecated-declarations")
target_compile_options(chisel_occt PRIVATE ${CHISEL_OCCT_OPT} -ffunction-sections -fdata-sections -fvisibility=hidden -fvisibility-inlines-hidden -Wall -Wextra)
if(CHISEL_OCCT_LTO STREQUAL "thin")
  target_compile_options(chisel_occt PRIVATE -flto=thin)
  target_link_options(chisel_occt PRIVATE -flto=thin)
endif()

foreach(tk ${CHISEL_OCCT_TKS})
  add_library(occt_static_${tk} STATIC IMPORTED)
  set_target_properties(occt_static_${tk} PROPERTIES IMPORTED_LOCATION "${CHISEL_OCCT_ARCHIVE_DIR}/lib/lib${tk}.a")
  target_link_libraries(chisel_occt PRIVATE occt_static_${tk})
endforeach()
target_link_libraries(chisel_occt PRIVATE log dl)   # c++_shared comes from ANDROID_STL; never libTK*.so
target_link_options(chisel_occt PRIVATE
  "LINKER:--gc-sections" "LINKER:--icf=all" "LINKER:--exclude-libs,ALL" "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/exports.map"
  "LINKER:-z,max-page-size=16384" "LINKER:-z,common-page-size=16384")
set_target_properties(chisel_occt PROPERTIES LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/exports.map")
