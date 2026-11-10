// SPDX-License-Identifier: LGPL-2.1-only OR LicenseRef-Chisel-Project
// Chisel 3D OCCT kernel interface header. Part of ksughosh/chisel-3d-occt (public, LGPL-2.1-or-later).
// This header defines the IGeometryKernel abstraction implemented by the OCCT adapter.
// No OCCT types cross this boundary — pure C++ + glm only.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace chisel {

using V2 = glm::dvec2;
using V3 = glm::dvec3;

struct Uid {
  uint64_t id = 0;
  bool operator==(const Uid& o) const { return id == o.id; }
  bool operator!=(const Uid& o) const { return id != o.id; }
  bool operator<(const Uid& o) const { return id < o.id; }
  static Uid fromHex(const std::string& hex);
  std::string hex() const;
};

struct PlaneFrame {
  V3 origin{0, 0, 0};
  V3 xAxis{1, 0, 0};
  V3 yAxis{0, 1, 0};
  V3 normal{0, 0, 1};
};

enum class Role : uint8_t { Side, CapStart, CapEnd, RevSurface, RevCapStart, RevCapEnd, Seam };

struct TopoName {
  Uid feature;
  Role role = Role::Side;
  Uid source;
  uint16_t split = 0;
  std::string str() const;
  auto operator<=>(const TopoName&) const = default;
};

struct EdgeName { TopoName a, b; uint8_t tie = 0; };

struct NameMap {
  std::vector<TopoName> faces;
  std::vector<EdgeName> edges;
  std::optional<int> find(const TopoName&) const;
};

struct BodyHandle { uint64_t id = 0; bool valid() const { return id != 0; } };

struct Loop {
  enum class Kind : uint8_t { Line, Circle };
  Kind kind = Kind::Line;
  V2 a{0, 0}, b{0, 0};
  V2 c{0, 0};
  double r = 0;
  bool ccw = true;
};

struct Region {
  std::vector<Loop> outer;
  std::vector<std::vector<Loop>> holes;
  double areaMm2 = 0;
  V2 centroid{0};
};

struct Profile2D {
  PlaneFrame plane;
  std::vector<Region> regions;
  double areaMm2 = 0;
  V2 centroid{0};
};

struct ExtrudeSpec { double distanceMm = 0; bool symmetric = false; bool flip = false; };
struct RevolveSpec { V3 axisOrigin{0, 0, 0}; V3 axisDir{1, 0, 0}; double angleRad = 2 * 3.14159265358979323846; };

struct MeshData {
  std::vector<float> pos, nrm;
  std::vector<uint32_t> idx;
  std::vector<uint32_t> triFace;
  std::vector<float> edgePos;
  std::vector<uint32_t> edgeId;
  uint64_t version = 0;
  std::vector<uint32_t> faceTriStart;
  std::vector<uint32_t> faceEdgeStart;
  std::vector<uint32_t> edgeFace;
  size_t triangleCount() const { return idx.size() / 3; }
  bool hasFaceRanges() const { return !faceTriStart.empty(); }
};

struct Aabb { V3 min{0}, max{0}; };

struct KernelResult {
  bool ok = false;
  std::string error;
  std::string fix;
  BodyHandle body;
  NameMap names;
};

struct MassProps { double volumeMm3 = 0; V3 centroid{0}; };

struct KernelCaps {
  bool booleans = false;
  bool intersect = false;
  bool toFace = false;
  bool exactBrep = false;
  bool namesThroughBooleans = false;
  bool copy = false;
};

enum class BoolOp : uint8_t { NewBody, Join, Cut };

class IGeometryKernel {
 public:
  virtual ~IGeometryKernel() = default;
  virtual const char* name() const = 0;
  virtual KernelCaps caps() const = 0;
  virtual KernelResult extrude(const Profile2D&, const ExtrudeSpec&, Uid feature) = 0;
  virtual KernelResult revolve(const Profile2D&, const RevolveSpec&, Uid feature) = 0;
  virtual KernelResult boolean(BoolOp, BodyHandle target, BodyHandle tool, Uid feature) = 0;
  virtual MassProps massProps(BodyHandle) = 0;
  virtual bool validate(BodyHandle, std::string* why) = 0;
  virtual std::shared_ptr<const MeshData> tessellate(BodyHandle, double chordTolMm, double angTolRad) = 0;
  virtual std::optional<PlaneFrame> planarFace(BodyHandle, const TopoName&) = 0;
  virtual std::optional<Profile2D> faceProfile(BodyHandle, const TopoName&) = 0;
  virtual std::vector<V3> snapPoints(BodyHandle) = 0;
  virtual void release(BodyHandle) = 0;
  virtual KernelResult copyBody(BodyHandle, Uid /*feature*/) {
    KernelResult r; r.ok = false; r.error = "not implemented"; r.fix = ""; return r;
  }
  virtual std::optional<Aabb> bounds(BodyHandle) { return std::nullopt; }
};

std::unique_ptr<IGeometryKernel> makeKernel();
std::unique_ptr<IGeometryKernel> makeMeshKernel();

double meshSignedVolume(const MeshData&);

}  // namespace chisel
