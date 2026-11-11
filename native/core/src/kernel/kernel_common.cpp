// SPDX-License-Identifier: LGPL-2.1-only
// Shared kernel helpers (KERN-2, spec 16 8.2a): the only core definitions libchisel_occt.so needs besides occt_kernel.cpp, occt_entry.cpp and uid.cpp.
// Keep this file free of sketch, document and JSON dependencies: it is part of the published LGPL source set.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "chisel/kernel/geometry_kernel.h"

namespace chisel {

// Signed sweep (rad) of one loop piece in its traversal direction; a full circle is +-2 pi. (profile_detector.cpp keeps its own copy for tessellateLoop.)
static double signedSweep(const LoopEdge& e) {
  constexpr double kTwoPi = 2 * M_PI;
  double a0 = std::atan2(e.a.y - e.c.y, e.a.x - e.c.x), a1 = std::atan2(e.b.y - e.c.y, e.b.x - e.c.x);
  if (e.kind == LoopEdge::Circle) return e.ccw ? kTwoPi : -kTwoPi;
  double d = a1 - a0;
  if (e.ccw) { while (d <= 1e-12) d += kTwoPi; while (d > kTwoPi + 1e-12) d -= kTwoPi; }
  else { while (d >= -1e-12) d -= kTwoPi; while (d < -kTwoPi - 1e-12) d += kTwoPi; }
  return d;
}

double loopSignedArea(const Loop& loop) {
  double A = 0;
  for (auto& e : loop) {
    A += 0.5 * (e.a.x * e.b.y - e.b.x * e.a.y);
    if (e.kind != LoopEdge::Line) { double th = signedSweep(e); A += 0.5 * e.r * e.r * (th - std::sin(th)); }
  }
  return A;
}

static const char* roleName(Role r) {
  static const char* n[] = {"Side", "CapStart", "CapEnd", "RevSurface", "RevCapStart", "RevCapEnd", "Seam"};
  return n[static_cast<int>(r)];
}

std::string TopoName::str() const {
  char buf[96];
  std::snprintf(buf, sizeof buf, "%s.%s(%s)#%u", feature.hex().substr(0, 8).c_str(), roleName(role), source.hex().substr(0, 8).c_str(), split);
  return buf;
}

std::optional<int> NameMap::find(const TopoName& n) const {
  for (size_t i = 0; i < faces.size(); ++i) if (faces[i] == n) return (int)i;
  return std::nullopt;
}

void buildFaceRanges(MeshData& m, uint32_t nFaces, const std::vector<uint32_t>& ef) {
  size_t nt = m.idx.size() / 3;
  uint32_t maxF = 0; for (auto f : m.triFace) maxF = std::max(maxF, f);
  if (!m.triFace.empty()) nFaces = std::max<uint32_t>(nFaces, maxF + 1);
  // triangles: stable counting sort by face
  m.faceTriStart.assign((size_t)nFaces + 1, 0);
  for (size_t t = 0; t < nt && t < m.triFace.size(); ++t) ++m.faceTriStart[m.triFace[t] + 1];
  for (size_t f = 0; f < nFaces; ++f) m.faceTriStart[f + 1] += m.faceTriStart[f];
  if (m.triFace.size() == nt) {
    std::vector<uint32_t> cursor(m.faceTriStart.begin(), m.faceTriStart.end() - 1), idx(m.idx.size()), tf(nt);
    for (size_t t = 0; t < nt; ++t) { uint32_t at = cursor[m.triFace[t]]++; idx[at * 3] = m.idx[t * 3]; idx[at * 3 + 1] = m.idx[t * 3 + 1]; idx[at * 3 + 2] = m.idx[t * 3 + 2]; tf[at] = m.triFace[t]; }
    m.idx = std::move(idx); m.triFace = std::move(tf);
  }
  // edge segments: stable sort by lowest adjacent face
  size_t ne = m.edgeId.size();
  m.faceEdgeStart.assign((size_t)nFaces + 1, 0); m.edgeFace.clear();
  if (ne == 0 || ef.size() != ne * 2 || m.edgePos.size() != ne * 6) return;
  std::vector<uint32_t> order(ne); for (size_t i = 0; i < ne; ++i) order[i] = (uint32_t)i;
  auto key = [&](uint32_t i) { return std::min(ef[i * 2], ef[i * 2 + 1]); };
  std::stable_sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) { return key(a) < key(b); });
  std::vector<float> ep(ne * 6); std::vector<uint32_t> eid(ne); m.edgeFace.resize(ne * 2);
  for (size_t k = 0; k < ne; ++k) {
    uint32_t i = order[k]; std::copy_n(&m.edgePos[i * 6], 6, &ep[k * 6]); eid[k] = m.edgeId[i]; m.edgeFace[k * 2] = ef[i * 2]; m.edgeFace[k * 2 + 1] = ef[i * 2 + 1];
    uint32_t kf = key(i); if (kf < nFaces) ++m.faceEdgeStart[(size_t)kf + 1];
  }
  for (size_t f = 0; f < nFaces; ++f) m.faceEdgeStart[f + 1] += m.faceEdgeStart[f];
  m.edgePos = std::move(ep); m.edgeId = std::move(eid);
}

}  // namespace chisel
