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
// KERN-5 (ADR-027, ABI 2): `chamfer` and `draft` are separate caps (ADR-023: each op by its own cap, never `exactBrep`). Both are appended at the end.
struct KernelCaps { bool booleans = false, intersect = false, toFace = false, exactBrep = false, namesThroughBooleans = false, copy = false, chamfer = false, draft = false, bend = false, stepExport = false; };

// KERN-5 (G2 spec 4.1b/4.1c). Equal-distance chamfer of one or more edges. Each edge is named by NameMap::edges of the body (the two adjacent face names
// plus `tie`, the rank among edges with the same face pair, sorted by edge midpoint). distanceMm >= 1e-3 (kMinChamferMm); every failure is an error, never a silent no-op.
struct ChamferSpec { std::vector<EdgeName> edges; double distanceMm = 0; };
// Face draft (taper). Every face in `faces` tilts about the neutral plane (through neutralOrigin, normal = pullDir). angleRad > 0 tilts the faces INWARD (the body
// loses material), < 0 outward; |angleRad| in [1e-4, 1.5]. The kernel checks that the volume change has the requested sign and reports an error when it has not
// (e.g. pullDir pointing the wrong way). For a drag-in on a picked edge E between faces Ft (tilting) and Fo: faces = {Ft}, neutralOrigin = a point of E,
// pullDir = -(outward normal of Fo).
struct DraftSpec { std::vector<TopoName> faces; glm::dvec3 pullDir{0, 0, 1}; glm::dvec3 neutralOrigin{0}; double angleRad = 0; };
// F3 (G3 spec 5.3). Two-point bend of the zone between two planes. Frame: `axisOrigin` + `axisDir` = bend axis (perpendicular to the bend plane); `alongDir` (unit, perpendicular
// to axisDir) points from anchor A to anchor B; the zone is s in [zoneStart, zoneEnd] along alongDir measured from axisOrigin; `neutralOffsetMm` = distance of the neutral surface
// from the reference surface (length is preserved there: R_n = zoneLength / angleRad). angleRad is signed. cells = 0: the kernel chooses N (G3 5.5a); > 0: use N.
struct BendSpec { glm::dvec3 axisOrigin{0}, axisDir{0, 1, 0}, alongDir{1, 0, 0}; double zoneStart = 0, zoneEnd = 0, neutralOffsetMm = 0, angleRad = 0; int cells = 0; bool preferExact = true; double targetDeviationMm = 0.01; };
// What the bend actually did (the document turns it into bendDeviationMm / bendExact, G3 5.1). deviation fields are RAW; the document applies the safety factor and rounding.
struct BendReport { bool exact = false, boundKnown = false; double devSurfMm = 0, maxEdgeTolMm = 0, volumeMm3 = 0; int cells = 0; int degree = 1; };
// F6c STEP (G4 spec 8.1c). Exact B-rep export of whole bodies; the kernel writes the file at `path` (the core never picks paths; partial file removed on failure).
// `schema` is "AP214IS" in the product (the UI offers nothing else); the other names exist only so tests can pin the Interface_Static ordering bug (8.1c 4).
struct StepBody { BodyHandle body; std::string name; };
struct StepSpec { std::vector<StepBody> bodies; std::string path; std::string schema = "AP214IS"; };
struct StepResult { bool ok = false; std::string error, fix; uint64_t bytes = 0; int bodies = 0; };

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
  // KERN-5 CONTRACT defaults: honest "not implemented" (caps().chamfer / caps().draft stay false). The OCCT kernel overrides both. The input body is never modified;
  // the result is a NEW body whose names carry over (faces merged by the lazy unify take the smallest predecessor name, G2 4.1c).
  virtual KernelResult chamferEdges(BodyHandle, const ChamferSpec&, Uid /*feature*/) { KernelResult r; r.ok = false; r.error = "not implemented"; r.fix = ""; return r; }
  virtual KernelResult draftFaces(BodyHandle, const DraftSpec&, Uid /*feature*/) { KernelResult r; r.ok = false; r.error = "not implemented"; r.fix = ""; return r; }
  // F3 CONTRACT default: honest "not implemented" (caps().bend stays false). Input body never modified; the result is a NEW body, faces map one-to-one (names carry over).
  virtual KernelResult bendBody(BodyHandle, const BendSpec&, Uid /*feature*/, BendReport* /*out*/) { KernelResult r; r.ok = false; r.error = "not implemented"; r.fix = ""; return r; }
  // F6c CONTRACT default: honest "not implemented" (caps().stepExport stays false until KERN-6).
  virtual StepResult exportStep(const StepSpec&) { StepResult r; r.error = "not implemented"; return r; }
};
// Reorders triangles face-contiguously and fills faceTriStart/faceEdgeStart/edgeFace. nFaces = number of mesh-local face ids (>= max triFace + 1).
// edgeFacePairs: 2 x uint32 per edge segment (adjacent mesh-local faces, UINT32_MAX = none), parallel to edgeId / edgePos (6 floats each).
void buildFaceRanges(MeshData&, uint32_t nFaces, const std::vector<uint32_t>& edgeFacePairs);

}  // namespace chisel
