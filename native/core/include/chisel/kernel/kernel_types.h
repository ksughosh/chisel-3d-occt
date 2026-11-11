// SPDX-License-Identifier: LGPL-2.1-only OR LicenseRef-Chisel-Project
// Plain geometry value types shared by the sketch layer and the geometry kernel ABI (KERN-2, spec 16 8.2a).
// Part of the published LGPL interface (with geometry_kernel.h and common/uid.h): changing it requires bumping chisel_occt_abi.
#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "chisel/common/uid.h"

namespace chisel {

struct PlaneFrame { glm::dvec3 origin{0}, xAxis{1, 0, 0}, yAxis{0, 1, 0}, normal{0, 0, 1}; };

// One traversed piece of a region loop (in traversal direction).
struct LoopEdge {
  enum Kind { Line, Arc, Circle } kind = Line;
  glm::dvec2 a{0}, b{0};   // start and end of the piece in traversal direction
  glm::dvec2 c{0};         // centre (arcs and circles)
  double r = 0;
  bool ccw = true;         // traversal direction about c
  Uid source;              // sketch entity UID
};
using Loop = std::vector<LoopEdge>;

double loopSignedArea(const Loop&);   // analytic (defined in src/kernel/kernel_common.cpp)

}  // namespace chisel
