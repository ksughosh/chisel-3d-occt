// SPDX-License-Identifier: LGPL-2.1-only
// KERN-5 draft (G2 spec 4.1b guards, 4.1c lazy unify). BRepOffsetAPI_DraftAngle on a COPY of the input body.
// Convention (spike + QA VerifyA, kernel fact): Direction = pull direction, pointing from the neutral plane INTO the body along the drafted faces;
// a POSITIVE angle then tilts the face inward (the body loses material). The kernel verifies that sign on every result.
#include <algorithm>
#include <cmath>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include "occt_bevel_guard.h"

namespace chisel::bevel {
namespace {
const char* kE = "The taper could not be made.";
const char* kF = "Try a smaller angle.";
const char* kLostE = "This face cannot be tapered.";   // the face vanished in the unify or does not resolve
const char* kLostF = "Pick a different face.";

std::vector<int> resolveFaces(const Input& in, const DraftSpec& spec) {
  std::vector<int> idx;
  for (auto& n : spec.faces) {
    int hit = -1; for (size_t i = 0; i < in.faceNames.size(); ++i) if (in.faceNames[i] == n) { hit = (int)i; break; }
    if (hit < 0) return {};
    if (std::find(idx.begin(), idx.end(), hit) == idx.end()) idx.push_back(hit);
  }
  return idx;
}

std::vector<TopoName> nameResult(BRepOffsetAPI_DraftAngle& da, const Work& w, const TopTools_IndexedMapOfShape& rf, Uid feature) {
  std::vector<TopoName> names(rf.Extent(), TopoName{feature, Role::Side, {}, 0}); std::vector<bool> done(rf.Extent(), false);
  for (int i = 1; i <= w.faces.Extent(); ++i) {
    TopoDS_Shape s = da.ModifiedShape(w.faces(i)); int k = s.IsNull() ? 0 : rf.FindIndex(s);
    if (!k) k = rf.FindIndex(w.faces(i));
    if (k && !done[k - 1]) { names[k - 1] = w.faceNames[i - 1]; done[k - 1] = true; }
  }
  makeNamesUnique(names); return names;
}
}  // namespace

Out draft(const Input& in, const DraftSpec& spec, Uid feature) {
  try {
    if (!std::isfinite(spec.angleRad) || !finiteV(spec.pullDir) || !finiteV(spec.neutralOrigin) || glm::length(spec.pullDir) < 1e-12) return refuse(kE, "Enter a valid angle.");
    double a = std::abs(spec.angleRad); if (a < kMinDraftRad || a > kMaxDraftRad) return refuse("The taper is too small or too large to make.", "Use an angle between 0.01° and 85°.");
    std::vector<int> idx = resolveFaces(in, spec); if (idx.empty()) return refuse(kLostE, kLostF);
    Work w; if (!makeWork(in.shape, in.faces, in.edges, in.faceNames, w)) return refuse(kE, kF);
    glm::dvec3 d = glm::normalize(spec.pullDir); gp_Dir dir(d.x, d.y, d.z); gp_Pln neutral(gp_Pnt(spec.neutralOrigin.x, spec.neutralOrigin.y, spec.neutralOrigin.z), dir);
    double v0 = volumeGK(w.shape); int f0 = w.faces.Extent();
    BRepOffsetAPI_DraftAngle da(w.shape);
    for (int i : idx) {
      int k = (size_t)i < w.faceOfInput.size() ? w.faceOfInput[i] : -1; if (k < 0) return refuse(kLostE, kLostF);
      da.Add(TopoDS::Face(w.faces(k + 1)), dir, spec.angleRad, neutral); if (!da.AddDone()) return refuse(kE, kF);
    }
    da.Build(); if (!da.IsDone()) return refuse(kE, kF);
    TopoDS_Shape r = da.Shape(); if (!isValid(r)) return refuse(kE, kF);   // a draft past a hole break-through is ok-but-INVALID (kernel fact)
    double v1 = volumeGK(r), eps = 1e-9 * std::max(1.0, v0), dv = v1 - v0;
    bool signOk = spec.angleRad > 0 ? dv < -eps : dv > eps;   // inward removes material; a wrong pull direction flips it silently in OCCT (measured)
    TopTools_IndexedMapOfShape rf; TopExp::MapShapes(r, TopAbs_FACE, rf);
    if (!signOk || rf.Extent() != f0) return refuse(kE, kF);
    Out o; o.ok = true; o.shape = r; o.faceNames = nameResult(da, w, rf, feature); return o;
  } catch (const Standard_Failure&) { return refuse(kE, kF); }
  catch (...) { return refuse(kE, kF); }
}

}  // namespace chisel::bevel
