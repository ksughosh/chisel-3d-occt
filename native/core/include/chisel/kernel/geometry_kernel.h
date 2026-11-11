// SPDX-License-Identifier: LGPL-2.1-only OR LicenseRef-Chisel-Project
#pragma once
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "chisel/common/uid.h"
#include "chisel/kernel/kernel_types.h"

namespace chisel {

enum class BoolOp : uint8_t { NewBody, Join, Cut, Intersect };
enum class Role : uint8_t { Side, CapStart, CapEnd, RevSurface, RevCapStart, RevCapEnd, Seam };

struct TopoName {
  Uid feature; Role role = Role::Side; Uid source; uint16_t split = 0;
  std::string str() const;   // "ab12cd34.Side(1f2e3d4c)#0" for user messages
  auto operator<=>(const TopoName&) const = default;
};
struct EdgeName { TopoName a, b; uint8_t tie = 0; };
struct NameMap {
  std::vector<TopoName> faces; std::vector<EdgeName> edges;
  std::optional<int> find(const TopoName&) const;
};

struct BodyHandle { uint64_t id = 0; bool valid() const { return id != 0; } };

struct Profile2D {
  PlaneFrame plane;
  std::vector<Loop> loops;   // [0] outer CCW, rest holes CW
  double areaMm2 = 0; glm::dvec2 centroid{0};
};
struct ExtrudeSpec { double distanceMm = 0; bool symmetric = false; bool flip = false; };
struct RevolveSpec { glm::dvec3 axisOrigin{0}, axisDir{1, 0, 0}; double angleRad = 2 * M_PI; };

struct MeshData {
  std::vector<float> pos, nrm; std::vector<uint32_t> idx; std::vector<uint32_t> triFace;
  std::vector<float> edgePos; std::vector<uint32_t> edgeId; uint64_t version = 0;
  // Face ranges (spec 07 6.4, spec 10 4.2), produced by both kernels since M2-C (buildFaceRanges). Triangles are stored face-contiguous and
  // triFace stays parallel. Edge segments are stored sorted by their lowest adjacent face; segments with no adjacent face (open) trail after the last range.
  // Empty means "ranges not produced" (e.g. the debug perf mesh); consumers must fall back to the Run 1 path, never treat empty as "zero faces".
  std::vector<uint32_t> faceTriStart;   // size faces+1: triangles of face f are [faceTriStart[f], faceTriStart[f+1])
  std::vector<uint32_t> faceEdgeStart;  // size faces+1: edge segments of face f
  std::vector<uint32_t> edgeFace;       // 2 x uint32 per edge segment: the two adjacent mesh-local face ids (UINT32_MAX when open)
  size_t triangleCount() const { return idx.size() / 3; }
  bool hasFaceRanges() const { return !faceTriStart.empty(); }
};
struct Aabb { glm::dvec3 min{0}, max{0}; };
struct KernelResult { bool ok = false; std::string error, fix; BodyHandle body; NameMap names; };
struct MassProps { double volumeMm3 = 0; glm::dvec3 centroid{0}; };

// What a kernel can do (spec 10 4.3 capability matrix). The UI is capability driven (Join/Cut disabled with a tooltip when !booleans).
struct KernelCaps { bool booleans = false, intersect = false, toFace = false, exactBrep = false, namesThroughBooleans = false, copy = false; };

class IGeometryKernel {
 public:
  virtual ~IGeometryKernel() = default;
  virtual const char* name() const = 0;   // "mesh-L0" | "occt-8.0.1"
  virtual KernelCaps caps() const = 0;
  virtual KernelResult extrude(const Profile2D&, const ExtrudeSpec&, Uid feature) = 0;
  virtual KernelResult revolve(const Profile2D&, const RevolveSpec&, Uid feature) = 0;
  virtual KernelResult boolean(BoolOp, BodyHandle target, BodyHandle tool, Uid feature) = 0;  // L0: ok=false
  virtual MassProps massProps(BodyHandle) = 0;
  virtual bool validate(BodyHandle, std::string* why) = 0;
  virtual std::shared_ptr<const MeshData> tessellate(BodyHandle, double chordTolMm, double angTolRad) = 0;
  virtual std::optional<PlaneFrame> planarFace(BodyHandle, const TopoName&) = 0;
  virtual std::optional<Profile2D> faceProfile(BodyHandle, const TopoName&) = 0;
  virtual std::vector<glm::dvec3> snapPoints(BodyHandle) = 0;
  virtual void release(BodyHandle) = 0;
  // M2-0 CONTRACT defaults: honest "not implemented". M2-C overrides both kernels (and sets caps().copy).
  virtual KernelResult copyBody(BodyHandle, Uid /*feature*/) { KernelResult r; r.ok = false; r.error = "not implemented"; r.fix = ""; return r; }
  virtual std::optional<Aabb> bounds(BodyHandle) { return std::nullopt; }
};
// Reorders triangles face-contiguously and fills faceTriStart/faceEdgeStart/edgeFace. nFaces = number of mesh-local face ids (>= max triFace + 1).
// edgeFacePairs: 2 x uint32 per edge segment (adjacent mesh-local faces, UINT32_MAX = none), parallel to edgeId / edgePos (6 floats each).
void buildFaceRanges(MeshData&, uint32_t nFaces, const std::vector<uint32_t>& edgeFacePairs);

}  // namespace chisel
