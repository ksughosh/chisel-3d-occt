# SPDX-License-Identifier: LGPL-2.1-only OR LicenseRef-Chisel-Project
# Standalone OCCT kernel build for Chisel 3D.
# Produces libchisel_occt.so (Android ARM64) or libchisel_occt.dylib (macOS host).

cmake_minimum_required(VERSION 3.22)
project(chisel_occt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(CHISEL_HOST "Host build (macOS)" OFF)

set(GLM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glm" CACHE PATH "GLM include dir")

find_package(OCCT REQUIRED CONFIG)
include_directories(${OCCT_INCLUDE_DIRS})

set(CORE_SRC
  src/kernel/occt_kernel.cpp
  src/kernel/uid.cpp
)

add_library(chisel_occt SHARED ${CORE_SRC})
target_include_directories(chisel_occt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(chisel_occt SYSTEM PRIVATE ${GLM_INCLUDE_DIR} ${OCCT_INCLUDE_DIRS})

target_compile_definitions(chisel_occt PUBLIC CHISEL_WITH_OCCT=1 CHISEL_KERNEL_NAME="occt-8.0.1")
target_compile_options(chisel_occt PRIVATE -Wall -Wextra -Werror -Wno-deprecated-declarations)

set(OCCT_TKS TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool TKShHealing TKMesh)
foreach(tk ${OCCT_TKS})
  target_link_libraries(chisel_occt PRIVATE OCCT::${tk})
endforeach()

if(CHISEL_HOST)
  set_target_properties(chisel_occt PROPERTIES INTERFACE_LINK_OPTIONS "-Wl,-rpath,\\$ORIGIN/../lib")
endif()

install(TARGETS chisel_occt DESTINATION lib)
install(FILES include/chisel/kernel/geometry_kernel.h DESTINATION include/chisel/kernel)
