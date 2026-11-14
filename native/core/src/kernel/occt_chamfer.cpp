// SPDX-License-Identifier: LGPL-2.1-only
// KERN-5 chamfer (G2 spec 4.1b guards, 4.1c lazy unify). BRepFilletAPI_MakeChamfer, equal distance, on a COPY of the input body.
#include <algorithm>
#include <cmath>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp_Face.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include "occt_bevel_guard.h"

namespace chisel::bevel {
namespace {
const char* kGenericE = "The chamfer could not be made on this edge.";
const char* kGenericF = "Try a smaller distance or fewer edges.";
const char* kSeamE = "This edge cannot be chamfered.";   // S4 (approved)
const char* kSeamF = "Pick a different edge.";
const char* kRefE = "Reference lost: the face.";   // W4 F-3 (spec 22 §5.1)
const char* kRefF = "Pick the edge again.";
const char* kAngE = "The angle must be between 0 and 90 degrees.";
const char* kAngF = "Type an angle such as 45.";
constexpr double kMinAngle = 1e-4;
const char* kSteepE = "The angle is too steep for this edge.";
const char* kSteepF = "Use a smaller angle.";

// Input edge indices (0-based) of the requested names; false when a name does not resolve or repeats.
bool resolveEdges(const Input& in, const ChamferSpec& spec, std::vector<int>& idx) {
  for (auto& n : spec.edges) {
    int hit = -1;
    for (size_t i = 0; i < in.edgeNames.size(); ++i) if (sameEdgeName(in.edgeNames[i], n)) { hit = (int)i; break; }
    if (hit < 0) return false;
    if (std::find(idx.begin(), idx.end(), hit) == idx.end()) idx.push_back(hit);
  }
  return !idx.empty();
}

// Working edges of the requested input edges. Seam / smooth / unclassifiable edges are refused (4.1c). convexSign: +1 all convex, -1 all concave, 0 mixed.
bool mapToWork(const Work& w, const std::vector<int>& idx, std::vector<TopoDS_Edge>& out, int& convexSign) {
  TopTools_IndexedDataMapOfShapeListOfShape anc; TopExp::MapShapesAndAncestors(w.shape, TopAbs_EDGE, TopAbs_FACE, anc);
  bool anyVex = false, anyCave = false;
  for (int i : idx) {
    int k = (size_t)i < w.edgeOfInput.size() ? w.edgeOfInput[i] : -1; if (k < 0) return false;   // the edge vanished in the unify: it was a seam
    TopoDS_Edge e = TopoDS::Edge(w.edges(k + 1));
    Convexity c = edgeConvexity(anc, e);
    if (c == Convexity::Smooth || c == Convexity::Other) return false;
    (c == Convexity::Convex ? anyVex : anyCave) = true;
    bool dup = false; for (auto& x : out) if (x.IsSame(e)) dup = true;
    if (!dup) out.push_back(e);
  }
  convexSign = anyVex && anyCave ? 0 : anyVex ? 1 : -1;
  return !out.empty();
}

// W4 F-3 (spec 22 §5.2): the working reference face of every requested edge (parallel to `es`, first occurrence wins).
// 1 = ok, 0 = an edge does not resolve (seam), -1 = a refFace is unknown or not adjacent to its edge.
int refFacesToWork(const Input& in, const Work& w, const ChamferSpec& spec, const std::vector<TopoDS_Edge>& es, std::vector<TopoDS_Face>& fs) {
  TopTools_IndexedDataMapOfShapeListOfShape anc; TopExp::MapShapesAndAncestors(w.shape, TopAbs_EDGE, TopAbs_FACE, anc);
  fs.assign(es.size(), TopoDS_Face()); std::vector<bool> set(es.size(), false);
  for (size_t i = 0; i < spec.edges.size(); ++i) {
    int hit = -1;
    for (size_t j = 0; j < in.edgeNames.size(); ++j) if (sameEdgeName(in.edgeNames[j], spec.edges[i])) { hit = (int)j; break; }
    int k = hit >= 0 && (size_t)hit < w.edgeOfInput.size() ? w.edgeOfInput[hit] : -1; if (k < 0) return 0;
    const TopoDS_Shape& e = w.edges(k + 1); size_t slot = es.size();
    for (size_t s = 0; s < es.size(); ++s) if (es[s].IsSame(e)) slot = s;
    if (slot == es.size()) return 0;
    if (set[slot]) continue;
    int fi = -1;
    for (size_t j = 0; j < in.faceNames.size(); ++j) if (in.faceNames[j] == spec.refFaces[i]) { fi = (int)j; break; }
    int wf = fi >= 0 && (size_t)fi < w.faceOfInput.size() ? w.faceOfInput[fi] : -1; if (wf < 0) return -1;
    const TopoDS_Shape& f = w.faces(wf + 1); bool adj = false;
    if (anc.Contains(e)) for (const auto& x : anc.FindFromKey(e)) if (x.IsSame(f)) adj = true;
    if (!adj) return -1;
    fs[slot] = TopoDS::Face(f); set[slot] = true;
  }
  return 1;
}

// Outward normal angle between the two faces of e at its midpoint (pi - the interior dihedral angle theta of a convex edge); false when unknown.
bool normalTurn(const TopTools_IndexedDataMapOfShapeListOfShape& anc, const TopoDS_Edge& e, double& turn) {
  if (!anc.Contains(e) || anc.FindFromKey(e).Extent() != 2) return false;
  gp_Vec n[2]; int i = 0;
  for (const auto& x : anc.FindFromKey(e)) {
    const TopoDS_Face f = TopoDS::Face(x); double a = 0, b = 0;
    Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(e, f, a, b); if (pc.IsNull()) return false;
    gp_Pnt2d uv = pc->Value(0.5 * (a + b)); gp_Pnt p; BRepGProp_Face gf(f); gf.Normal(uv.X(), uv.Y(), p, n[i]);
    if (n[i].Magnitude() < 1e-12) return false; n[i].Normalize(); ++i;
  }
  turn = std::acos(std::clamp(n[0].Dot(n[1]), -1.0, 1.0)); return true;
}

}  // namespace
}  // namespace chisel::bevel

namespace chisel::bevel {
namespace {
// Names for the result: surviving faces keep the (merged) working name, faces generated from an edge are {feature, Side, edgeUid, 0}, anything else (vertex blends) {feature, Side, 0}.
std::vector<TopoName> nameResult(BRepFilletAPI_MakeChamfer& mk, const Work& w, const std::vector<TopoDS_Edge>& es, const TopTools_IndexedMapOfShape& rf, Uid feature) {
  std::vector<TopoName> names(rf.Extent(), TopoName{feature, Role::Side, {}, 0}); std::vector<bool> done(rf.Extent(), false);
  for (int i = 1; i <= w.faces.Extent(); ++i) {
    const TopoDS_Shape& f = w.faces(i); if (mk.IsDeleted(f)) continue;
    const auto& mod = mk.Modified(f); std::vector<int> ks;
    if (mod.IsEmpty()) { if (int k = rf.FindIndex(f)) ks.push_back(k); }
    else for (auto it = mod.begin(); it != mod.end(); ++it) if (int k = rf.FindIndex(*it)) ks.push_back(k);
    for (int k : ks) if (!done[k - 1]) { names[k - 1] = w.faceNames[i - 1]; done[k - 1] = true; }
  }
  for (auto& e : es) {
    int ei = w.edges.FindIndex(e); if (!ei) continue; Uid u = edgeUid(w.edgeNames[ei - 1]);
    const auto& gen = mk.Generated(e); int n = 0;
    for (auto it = gen.begin(); it != gen.end(); ++it) if (int k = rf.FindIndex(*it)) if (!done[k - 1]) { names[k - 1] = TopoName{feature, Role::Side, u, (uint16_t)n++}; done[k - 1] = true; }
  }
  makeNamesUnique(names); return names;
}
}  // namespace

Out chamfer(const Input& in, const ChamferSpec& spec, Uid feature) {
  try {
    const bool eq = spec.mode == ChamferMode::Equal;   // W4 F-3 (spec 22 §5.1-5.2, ADR-070): Equal = the ABI-4 path, unchanged
    if (eq ? !spec.refFaces.empty() : spec.refFaces.size() != spec.edges.size()) return refuse(kRefE, kRefF);
    if (!std::isfinite(spec.distanceMm)) return refuse("The chamfer could not be made on this edge.", "Enter a valid distance.");
    if (spec.distanceMm < kMinChamferMm) return refuse("The chamfer is too small to make.", "Use at least 0.001 mm.");
    if (spec.mode == ChamferMode::TwoDistances && !(std::isfinite(spec.distance2Mm) && spec.distance2Mm >= kMinChamferMm)) return refuse("The chamfer is too small to make.", "Use at least 0.001 mm.");
    if (spec.mode == ChamferMode::DistanceAngle && !(spec.angleRad > kMinAngle && spec.angleRad < M_PI / 2 - kMinAngle)) return refuse(kAngE, kAngF);
    std::vector<int> idx; if (!resolveEdges(in, spec, idx)) return refuse(kSeamE, kSeamF);
    Work w; if (!makeWork(in.shape, in.faces, in.edges, in.faceNames, w)) return refuse(kGenericE, kGenericF);
    std::vector<TopoDS_Edge> es; int sign = 0; if (!mapToWork(w, idx, es, sign)) return refuse(kSeamE, kSeamF);
    double v0 = volumeGK(w.shape); int f0 = w.faces.Extent();
    std::vector<TopoDS_Face> fs;
    if (!eq) { int rr = refFacesToWork(in, w, spec, es, fs); if (rr == 0) return refuse(kSeamE, kSeamF); if (rr < 0) return refuse(kRefE, kRefF); }
    if (spec.mode == ChamferMode::DistanceAngle) {   // spec 22 §5.4: theta + a < pi  <=>  a < (pi - theta) = the outward-normal turn
      TopTools_IndexedDataMapOfShapeListOfShape anc; TopExp::MapShapesAndAncestors(w.shape, TopAbs_EDGE, TopAbs_FACE, anc);
      for (auto& e : es) { double turn = 0; if (normalTurn(anc, e, turn) && !(spec.angleRad < turn - 1e-9)) return refuse(kSteepE, kSteepF); }
    }
    BRepFilletAPI_MakeChamfer mk(w.shape);
    for (size_t i = 0; i < es.size(); ++i) {
      if (eq) mk.Add(spec.distanceMm, es[i]);
      else if (spec.mode == ChamferMode::TwoDistances) mk.Add(spec.distanceMm, spec.distance2Mm, es[i], fs[i]);   // d1 on F (OCCT convention, pinned by a test)
      else mk.AddDA(spec.distanceMm, spec.angleRad, es[i], fs[i]);
    }
    mk.Build(); if (!mk.IsDone()) return refuse(kGenericE, kGenericF);
    TopoDS_Shape r = mk.Shape(); if (!isValid(r)) return refuse(kGenericE, kGenericF);   // ok-but-INVALID beyond an adjacent arc radius (kernel fact)
    double v1 = volumeGK(r), eps = 1e-9 * std::max(1.0, v0), dv = v1 - v0;
    bool effect = sign > 0 ? dv < -eps : sign < 0 ? dv > eps : std::abs(dv) > eps;   // convex removes, concave adds material
    TopTools_IndexedMapOfShape rf; TopExp::MapShapes(r, TopAbs_FACE, rf);
    if (!effect || rf.Extent() <= f0) return refuse(kGenericE, kGenericF);
    Out o; o.ok = true; o.shape = r; o.faceNames = nameResult(mk, w, es, rf, feature); return o;
  } catch (const Standard_Failure&) { return refuse(kGenericE, kGenericF); }
  catch (...) { return refuse(kGenericE, kGenericF); }
}

}  // namespace chisel::bevel
