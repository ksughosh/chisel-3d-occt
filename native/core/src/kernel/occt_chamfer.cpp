// SPDX-License-Identifier: LGPL-2.1-only
// KERN-5 chamfer (G2 spec 4.1b guards, 4.1c lazy unify). BRepFilletAPI_MakeChamfer, equal distance, on a COPY of the input body.
#include <algorithm>
#include <cmath>
#include <BRepFilletAPI_MakeChamfer.hxx>
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
    if (!std::isfinite(spec.distanceMm)) return refuse("The chamfer could not be made on this edge.", "Enter a valid distance.");
    if (spec.distanceMm < kMinChamferMm) return refuse("The chamfer is too small to make.", "Use at least 0.001 mm.");
    std::vector<int> idx; if (!resolveEdges(in, spec, idx)) return refuse(kSeamE, kSeamF);
    Work w; if (!makeWork(in.shape, in.faces, in.edges, in.faceNames, w)) return refuse(kGenericE, kGenericF);
    std::vector<TopoDS_Edge> es; int sign = 0; if (!mapToWork(w, idx, es, sign)) return refuse(kSeamE, kSeamF);
    double v0 = volumeGK(w.shape); int f0 = w.faces.Extent();
    BRepFilletAPI_MakeChamfer mk(w.shape); for (auto& e : es) mk.Add(spec.distanceMm, e);
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
