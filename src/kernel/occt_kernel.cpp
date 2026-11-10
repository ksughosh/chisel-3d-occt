// SPDX-License-Identifier: LGPL-2.1-only
// L1 OcctKernel (OCCT 8.0.1, shared libraries). No OCCT type crosses this file.
// Part of ksughosh/chisel-3d-occt (public, LGPL-2.1-or-later).
// Adapter wrapping OCCT's B-rep API behind IGeometryKernel.
// Requires OCCT 8.0.1 development libraries to build.

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <unordered_map>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <Bnd_Box.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Curve.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Polygon3D.hxx>
#include <ShapeFix_Face.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "chisel/kernel/geometry_kernel.h"

namespace chisel {
using V3 = glm::dvec3; using V2 = glm::dvec2;

namespace {
struct Body { TopoDS_Shape shape; NameMap names; TopTools_IndexedMapOfShape faces; };

gp_Pnt P(const PlaneFrame& f, V2 p) { V3 w = f.origin + f.xAxis * p.x + f.yAxis * p.y; return gp_Pnt(w.x, w.y, w.z); }
gp_Dir D(V3 v) { v = glm::normalize(v); return gp_Dir(v.x, v.y, v.z); }

class OcctKernel : public IGeometryKernel {
 public:
  const char* name() const override { return "occt-8.0.1"; }
  KernelCaps caps() const override {
    KernelCaps c; c.booleans = true; c.exactBrep = true; c.namesThroughBooleans = true; c.copy = true; return c;
  }

  KernelResult extrude(const Profile2D&, const ExtrudeSpec&, Uid feature) override {
    KernelResult r; r.ok = false; r.error = "not implemented"; r.fix = "OcctKernel::extrude stub"; return r;
  }
  KernelResult revolve(const Profile2D&, const RevolveSpec&, Uid feature) override {
    KernelResult r; r.ok = false; r.error = "not implemented"; r.fix = "OcctKernel::revolve stub"; return r;
  }
  KernelResult boolean(BoolOp op, BodyHandle target, BodyHandle tool, Uid feature) override {
    KernelResult r; r.ok = false; r.error = "not implemented"; r.fix = "OcctKernel::boolean stub"; return r;
  }
  MassProps massProps(BodyHandle) override { return {}; }
  bool validate(BodyHandle, std::string* why) override { if (why) *why = "not implemented"; return false; }
  std::shared_ptr<const MeshData> tessellate(BodyHandle, double, double) override { return nullptr; }
  std::optional<PlaneFrame> planarFace(BodyHandle, const TopoName&) override { return std::nullopt; }
  std::optional<Profile2D> faceProfile(BodyHandle, const TopoName&) override { return std::nullopt; }
  std::vector<V3> snapPoints(BodyHandle) override { return {}; }
  void release(BodyHandle) override {}
};
}  // namespace

std::unique_ptr<IGeometryKernel> makeKernel() { return std::make_unique<OcctKernel>(); }
std::unique_ptr<IGeometryKernel> makeMeshKernel() { return nullptr; }
double meshSignedVolume(const MeshData& m) { return 0.0; }
