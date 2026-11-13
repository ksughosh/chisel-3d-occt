// SPDX-License-Identifier: LGPL-2.1-only OR LicenseRef-Chisel-Project
// Plain geometry value types shared by the sketch layer and the geometry kernel ABI (KERN-2, spec 16 8.2a).
// Part of the published LGPL interface (with geometry_kernel.h and common/uid.h): changing it requires bumping chisel_occt_abi.
#pragma once
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "chisel/common/uid.h"

namespace chisel {

struct PlaneFrame { glm::dvec3 origin{0}, xAxis{1, 0, 0}, yAxis{0, 1, 0}, normal{0, 0, 1}; };

// ABI 4 (ADR-043, KERN-R3-0): exact data of a B-spline loop piece. Degree 3 for sketch splines, 2 for glyph outlines. `knots` is clamped (size poles + degree + 1);
// `weights` is empty for a non-rational curve. Shared (immutable) so a LoopEdge stays cheap to copy.
struct SplineData { int degree = 3; std::vector<double> knots; std::vector<glm::dvec2> poles; std::vector<double> weights; };

// One traversed piece of a region loop (in traversal direction).
// ABI 4 appends Ellipse, EllipseArc, BSpline (existing values keep their numbers). Ellipse: centre `c`, `axisU` = unit direction of the major axis, `r` = major radius,
// `r2` = minor radius, full curve (a == b is the start point). EllipseArc: the same, plus the eccentric-anomaly range [t0, t1] (radians, measured from axisU, ccw)
// traversed in the direction `ccw`. BSpline: `spline` (a and b are its first and last pole). Until an implementation lands no producer creates these kinds, and a
// kernel that does not support one must refuse the profile (never skip the edge).
struct LoopEdge {
  enum Kind { Line, Arc, Circle, Ellipse, EllipseArc, BSpline } kind = Line;
  glm::dvec2 a{0}, b{0};   // start and end of the piece in traversal direction
  glm::dvec2 c{0};         // centre (arcs and circles)
  double r = 0;
  bool ccw = true;         // traversal direction about c
  Uid source;              // sketch entity UID
  glm::dvec2 axisU{1, 0};  // Ellipse / EllipseArc: unit major-axis direction
  double r2 = 0;           // Ellipse / EllipseArc: minor radius
  double t0 = 0, t1 = 0;   // EllipseArc: parameter range
  std::shared_ptr<const SplineData> spline;   // BSpline
};
using Loop = std::vector<LoopEdge>;

double loopSignedArea(const Loop&);   // analytic (defined in src/kernel/kernel_common.cpp)

}  // namespace chisel
