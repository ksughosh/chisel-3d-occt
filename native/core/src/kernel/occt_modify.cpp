// SPDX-License-Identifier: LGPL-2.1-only
// FR-30 (spec 19 FR30 3.3; ADR-040 naming, ADR-041 op contract): fillet, shell, offset face and delete face on a COPY of the input body.
// Every op: lazy unify on the working copy (G2 4.1c), try/catch, IsDone, BRepCheck_Analyzer, a per-op effect check, a progress range that polls the
// caller's cancel flag and a time cap, and a structured KernelOpStatus. The document owns every user string (these texts are the developer fallback).
// Part of the published LGPL source set (ADR-039: new published file for the M5 ops).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffset_MakeOffset.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <ShapeFix_Solid.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include "occt_bevel_guard.h"

namespace chisel::bevel {
namespace {

// ---- cancel + time cap (ADR-041/042): OCCT polls UserBreak() from inside the algorithms through the progress range.
class CapProgress : public Message_ProgressIndicator {
 public:
  explicit CapProgress(const OpControl& c) : c_(c), t0_(std::chrono::steady_clock::now()) {}
  bool UserBreak() override {
    if (c_.cancelled && c_.cancelled(c_.user)) { cancelled_ = true; return true; }
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    if (c_.capSeconds > 0 && s > c_.capSeconds) { timedOut_ = true; return true; }
    return false;
  }
  void Show(const Message_ProgressScope&, const bool) override {}
  // Polled between steps too: a cancel that arrived while OCCT did not poll still wins.
  bool stop() { return UserBreak(); }
  KernelOpStatus why() const { return cancelled_ ? KernelOpStatus::Cancelled : KernelOpStatus::TimedOut; }
  bool broke() const { return cancelled_ || timedOut_; }
 private:
  OpControl c_; std::chrono::steady_clock::time_point t0_; bool cancelled_ = false, timedOut_ = false;
};
Out stopped(const CapProgress& p) {
  return p.why() == KernelOpStatus::Cancelled ? refuse(KernelOpStatus::Cancelled, "The operation was cancelled.", "Try again.")
                                              : refuse(KernelOpStatus::TimedOut, "The operation took too long.", "Try a simpler change.");
}

int inputEdgeIndex(const Input& in, const EdgeName& n) {
  for (size_t i = 0; i < in.edgeNames.size(); ++i) if (sameEdgeName(in.edgeNames[i], n)) return (int)i;
  return -1;
}
int inputFaceIndex(const Input& in, const TopoName& n) {
  for (size_t i = 0; i < in.faceNames.size(); ++i) if (in.faceNames[i] == n) return (int)i;
  return -1;
}
// Working faces of the requested names, deduplicated (two names merged by the unify are one face). False when a name does not resolve.
bool workFaces(const Input& in, const Work& w, const std::vector<TopoName>& names, std::vector<TopoDS_Face>& out) {
  for (auto& n : names) {
    const int i = inputFaceIndex(in, n); if (i < 0) return false;
    const int k = (size_t)i < w.faceOfInput.size() ? w.faceOfInput[i] : -1; if (k < 0) return false;
    TopoDS_Face f = TopoDS::Face(w.faces(k + 1)); bool dup = false;
    for (auto& x : out) if (x.IsSame(f)) dup = true;
    if (!dup) out.push_back(f);
  }
  return !out.empty();
}

// One solid out of an offset result: the algorithms may answer a closed shell (offset on face) or a compound holding one solid.
TopoDS_Shape oneSolid(const TopoDS_Shape& s) {
  if (s.IsNull()) return s;
  if (s.ShapeType() == TopAbs_SOLID) return s;
  int n = 0; TopoDS_Shape solid;
  for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) { ++n; solid = ex.Current(); }
  if (n == 1) return solid;
  if (n > 1) return TopoDS_Shape();
  int shells = 0; TopoDS_Shell sh;
  for (TopExp_Explorer ex(s, TopAbs_SHELL); ex.More(); ex.Next()) { ++shells; sh = TopoDS::Shell(ex.Current()); }
  if (shells != 1) return TopoDS_Shape();
  BRepBuilderAPI_MakeSolid ms(sh); if (!ms.IsDone()) return TopoDS_Shape();
  ShapeFix_Solid fx(ms.Solid()); fx.Perform(); return fx.Solid();
}

// Result names (ADR-040): a face of the result that IS a working face, or is its single Modified image, keeps the working name; faces Generated from
// an input shape get {feature, role, sourceOf(shape), n}; anything left gets {feature, role, nil, n}. Always unique.
template <class Mk>
std::vector<TopoName> nameByHistory(Mk& mk, const Work& w, const TopTools_IndexedMapOfShape& rf, Uid feature, Role genRole,
                                    const std::vector<std::pair<TopoDS_Shape, Uid>>& generators) {
  std::vector<TopoName> names(rf.Extent()); std::vector<bool> done(rf.Extent(), false);
  for (int i = 1; i <= w.faces.Extent(); ++i) {
    const TopoDS_Shape& f = w.faces(i);
    if (int k = rf.FindIndex(f)) { if (!done[k - 1]) { names[k - 1] = w.faceNames[i - 1]; done[k - 1] = true; } continue; }
    if (mk.IsDeleted(f)) continue;
    const auto& mod = mk.Modified(f);
    for (auto it = mod.begin(); it != mod.end(); ++it) if (int k = rf.FindIndex(*it)) if (!done[k - 1]) { names[k - 1] = w.faceNames[i - 1]; done[k - 1] = true; }
  }
  for (auto& [s, src] : generators) {
    const auto& gen = mk.Generated(s); uint16_t n = 0;
    for (auto it = gen.begin(); it != gen.end(); ++it) {
      TopTools_IndexedMapOfShape gf; TopExp::MapShapes(*it, TopAbs_FACE, gf);
      for (int j = 1; j <= gf.Extent(); ++j) if (int k = rf.FindIndex(gf(j))) if (!done[k - 1]) { names[k - 1] = TopoName{feature, genRole, src, n++}; done[k - 1] = true; }
    }
  }
  uint16_t rest = 0;
  for (int k = 0; k < rf.Extent(); ++k) if (!done[k]) names[k] = TopoName{feature, genRole, {}, rest++};
  makeNamesUnique(names); return names;
}

Out finish(const TopoDS_Shape& r, std::vector<TopoName> names) { Out o; o.ok = true; o.shape = r; o.faceNames = std::move(names); return o; }

// SHL-01 with zero opened faces (a closed hollow): offset the whole body by `off`, then subtract the inner solid (inward) or the original (outward).
// Faces of the original keep their names; faces of the offset copy are ShellInner sourced by the face they were offset from.
Out closedHollow(const Work& w, double off, Uid feature, CapProgress& pg, double v0) {
  const char* kE = "The body could not be hollowed."; const char* kF = "Try a smaller thickness or open fewer faces.";
  BRepOffsetAPI_MakeOffsetShape mo;
  mo.PerformByJoin(w.shape, off, 1e-6, BRepOffset_Skin, false, false, GeomAbs_Arc, false, pg.Start());
  if (pg.broke() || pg.stop()) return stopped(pg);
  if (!mo.IsDone()) return refuse(KernelOpStatus::Invalid, kE, kF);
  TopoDS_Shape o = oneSolid(mo.Shape()); if (o.IsNull() || !isValid(o)) return refuse(KernelOpStatus::Invalid, kE, kF);
  BRepAlgoAPI_Cut cut(off < 0 ? w.shape : o, off < 0 ? o : w.shape); cut.SetRunParallel(false);
  if (!cut.IsDone() || cut.HasErrors()) return refuse(KernelOpStatus::Invalid, kE, kF);
  TopoDS_Shape r = cut.Shape(); if (r.IsNull() || !isValid(r)) return refuse(KernelOpStatus::Invalid, kE, kF);
  const double v1 = volumeGK(r); if (!(v1 > 1e-9) || std::abs(v1 - v0) <= 1e-9 * std::max(1.0, v0)) return refuse(KernelOpStatus::NoEffect, kE, kF);
  TopTools_IndexedMapOfShape rf; TopExp::MapShapes(r, TopAbs_FACE, rf);
  std::vector<TopoName> names(rf.Extent()); std::vector<bool> done(rf.Extent(), false);
  auto mark = [&](const TopoDS_Shape& s, const TopoName& n) { bool any = false;
    if (int k = rf.FindIndex(s)) { if (!done[k - 1]) { names[k - 1] = n; done[k - 1] = true; } return; }
    for (auto it = cut.Modified(s).begin(); it != cut.Modified(s).end(); ++it) if (int k = rf.FindIndex(*it)) if (!done[k - 1]) { names[k - 1] = n; done[k - 1] = true; any = true; }
    (void)any; };
  for (int i = 1; i <= w.faces.Extent(); ++i) mark(w.faces(i), w.faceNames[i - 1]);
  for (int i = 1; i <= w.faces.Extent(); ++i) { uint16_t n = 0; const Uid src = nameUid(w.faceNames[i - 1]);
    for (const auto* l : {&mo.Modified(w.faces(i)), &mo.Generated(w.faces(i))}) for (auto it = l->begin(); it != l->end(); ++it) mark(*it, TopoName{feature, Role::ShellInner, src, n++}); }
  uint16_t rest = 0; for (int k = 0; k < rf.Extent(); ++k) if (!done[k]) names[k] = TopoName{feature, Role::ShellInner, {}, rest++};
  makeNamesUnique(names); return finish(r, std::move(names));
}
}  // namespace

// ---------------------------------------------------------------- fillet (FIL-01): constant radius, ChFi3d_Rational
Out fillet(const Input& in, const FilletSpec& spec, Uid feature, const OpControl& ctl) {
  const char* kE = "The fillet could not be made on this edge."; const char* kF = "Try a smaller radius or fewer edges.";
  try {
    if (!std::isfinite(spec.radiusMm)) return refuse(KernelOpStatus::Invalid, kE, "Enter a valid radius.");
    if (spec.radiusMm < kMinChamferMm) return refuse(KernelOpStatus::Invalid, "The fillet is too small to make.", "Use at least 0.001 mm.");
    if (spec.edges.empty()) return refuse(KernelOpStatus::Invalid, kE, "Pick an edge.");
    std::vector<int> idx;
    for (auto& n : spec.edges) { const int i = inputEdgeIndex(in, n); if (i < 0) return refuse(KernelOpStatus::Invalid, "Edge reference lost.", "Pick the edge again."); if (std::find(idx.begin(), idx.end(), i) == idx.end()) idx.push_back(i); }
    Work w; if (!makeWork(in.shape, in.faces, in.edges, in.faceNames, w)) return refuse(KernelOpStatus::Invalid, kE, kF);
    TopTools_IndexedDataMapOfShapeListOfShape anc; TopExp::MapShapesAndAncestors(w.shape, TopAbs_EDGE, TopAbs_FACE, anc);
    std::vector<TopoDS_Edge> es; std::vector<Uid> eu; bool vex = false, cave = false;
    for (int i : idx) {
      const int k = (size_t)i < w.edgeOfInput.size() ? w.edgeOfInput[i] : -1;
      if (k < 0) return refuse(KernelOpStatus::Invalid, "This edge cannot be rounded.", "Pick a different edge.");   // vanished in the unify: a seam
      TopoDS_Edge e = TopoDS::Edge(w.edges(k + 1)); const Convexity c = edgeConvexity(anc, e);
      if (c == Convexity::Smooth || c == Convexity::Other) return refuse(KernelOpStatus::Invalid, "This edge cannot be rounded.", "Pick a different edge.");
      (c == Convexity::Convex ? vex : cave) = true;
      bool dup = false; for (auto& x : es) if (x.IsSame(e)) dup = true;
      if (!dup) { es.push_back(e); eu.push_back(edgeUid(w.edgeNames[k])); }
    }
    const double v0 = volumeGK(w.shape); const int f0 = w.faces.Extent();
    occ::handle<CapProgress> pg = new CapProgress(ctl);
    BRepFilletAPI_MakeFillet mk(w.shape, ChFi3d_Rational);
    for (auto& e : es) mk.Add(spec.radiusMm, e);
    mk.Build(pg->Start());
    if (pg->broke() || pg->stop()) return stopped(*pg);
    if (!mk.IsDone()) return refuse(KernelOpStatus::Invalid, kE, kF);
    TopoDS_Shape r = mk.Shape(); if (!isValid(r)) return refuse(KernelOpStatus::Invalid, kE, kF);
    const double v1 = volumeGK(r), eps = 1e-9 * std::max(1.0, v0), dv = v1 - v0;
    const bool effect = vex && !cave ? dv < -eps : cave && !vex ? dv > eps : std::abs(dv) > eps;   // convex removes, concave adds material
    TopTools_IndexedMapOfShape rf; TopExp::MapShapes(r, TopAbs_FACE, rf);
    if (!effect || rf.Extent() <= f0) return refuse(KernelOpStatus::NoEffect, kE, kF);
    std::vector<std::pair<TopoDS_Shape, Uid>> gens; for (size_t i = 0; i < es.size(); ++i) gens.push_back({es[i], eu[i]});
    return finish(r, nameByHistory(mk, w, rf, feature, Role::FilletFace, gens));
  } catch (const Standard_Failure&) { return refuse(KernelOpStatus::Invalid, kE, kF); }
  catch (...) { return refuse(KernelOpStatus::Invalid, kE, kF); }
}

// ---------------------------------------------------------------- shell (SHL-01): open faces removed, wall of `thicknessMm` inward (default) or outward
Out shell(const Input& in, const ShellSpec& spec, Uid feature, const OpControl& ctl) {
  const char* kE = "The body could not be hollowed."; const char* kF = "Try a smaller thickness or open fewer faces.";
  try {
    if (!std::isfinite(spec.thicknessMm) || spec.thicknessMm < kMinChamferMm) return refuse(KernelOpStatus::Invalid, "The wall is too thin to make.", "Use at least 0.001 mm.");
    Work w; if (!makeWork(in.shape, in.faces, in.edges, in.faceNames, w)) return refuse(KernelOpStatus::Invalid, kE, kF);
    std::vector<TopoDS_Face> open;
    if (!spec.openFaces.empty() && !workFaces(in, w, spec.openFaces, open)) return refuse(KernelOpStatus::Invalid, "Face reference lost.", "Pick the face again.");
    if ((int)open.size() >= w.faces.Extent()) return refuse(KernelOpStatus::Invalid, kE, "Keep at least one face closed.");
    const double v0 = volumeGK(w.shape);
    occ::handle<CapProgress> pg = new CapProgress(ctl);
    const double off = spec.outward ? spec.thicknessMm : -spec.thicknessMm;
    if (open.empty()) return closedHollow(w, off, feature, *pg, v0);   // no opening: MakeThickSolid answers the offset solid itself (probe), so cut it out
    NCollection_List<TopoDS_Shape> closing; for (auto& f : open) closing.Append(f);
    BRepOffsetAPI_MakeThickSolid mk;
    mk.MakeThickSolidByJoin(w.shape, closing, off, 1e-6, BRepOffset_Skin, false, false, GeomAbs_Arc, false, pg->Start());
    if (pg->broke() || pg->stop()) return stopped(*pg);
    if (!mk.IsDone()) return refuse(KernelOpStatus::Invalid, kE, kF);
    TopoDS_Shape r = oneSolid(mk.Shape()); if (r.IsNull() || !isValid(r)) return refuse(KernelOpStatus::Invalid, kE, kF);
    const double v1 = volumeGK(r);
    if (!(v1 > 1e-9) || std::abs(v1 - v0) <= 1e-9 * std::max(1.0, v0) || (!spec.outward && !(v1 < v0))) return refuse(KernelOpStatus::NoEffect, kE, kF);
    TopTools_IndexedMapOfShape rf; TopExp::MapShapes(r, TopAbs_FACE, rf);
    for (auto& f : open) if (rf.FindIndex(f)) return refuse(KernelOpStatus::NoEffect, kE, kF);   // an opened face is still there
    // Kept outer faces keep their names; their offset copies (inner wall) are ShellInner sourced by the outer face's name (ADR-040).
    std::vector<std::pair<TopoDS_Shape, Uid>> gens;
    for (int i = 1; i <= w.faces.Extent(); ++i) gens.push_back({w.faces(i), nameUid(w.faceNames[i - 1])});
    std::vector<TopoName> names(rf.Extent()); std::vector<bool> done(rf.Extent(), false);
    for (int i = 1; i <= w.faces.Extent(); ++i) if (int k = rf.FindIndex(w.faces(i))) { names[k - 1] = w.faceNames[i - 1]; done[k - 1] = true; }
    for (int i = 1; i <= w.faces.Extent(); ++i) {   // the inner copy: Modified (offset image) or Generated, whichever OCCT reports
      const Uid src = nameUid(w.faceNames[i - 1]); uint16_t n = 0;
      for (const auto* l : {&mk.Modified(w.faces(i)), &mk.Generated(w.faces(i))})
        for (auto it = l->begin(); it != l->end(); ++it) if (int k = rf.FindIndex(*it)) if (!done[k - 1]) { names[k - 1] = TopoName{feature, Role::ShellInner, src, n++}; done[k - 1] = true; }
    }
    uint16_t rest = 0; for (int k = 0; k < rf.Extent(); ++k) if (!done[k]) names[k] = TopoName{feature, Role::ShellInner, {}, rest++};
    makeNamesUnique(names);
    return finish(r, std::move(names));
  } catch (const Standard_Failure&) { return refuse(KernelOpStatus::Invalid, kE, kF); }
  catch (...) { return refuse(KernelOpStatus::Invalid, kE, kF); }
}

// ---------------------------------------------------------------- offset face (OFS-01): picked faces move along their normal, neighbours extend or trim
Out offsetFaces(const Input& in, const OffsetFaceSpec& spec, Uid feature, const OpControl& ctl) {
  const char* kE = "This face cannot be offset."; const char* kF = "Try a smaller distance, or delete the face instead.";
  try {
    if (!std::isfinite(spec.distanceMm) || std::abs(spec.distanceMm) < kMinChamferMm) return refuse(KernelOpStatus::Invalid, "The distance is too small.", "Use at least 0.001 mm.");
    Work w; if (!makeWork(in.shape, in.faces, in.edges, in.faceNames, w)) return refuse(KernelOpStatus::Invalid, kE, kF);
    std::vector<TopoDS_Face> fs; if (!workFaces(in, w, spec.faces, fs)) return refuse(KernelOpStatus::Invalid, "Face reference lost.", "Pick the face again.");
    const double v0 = volumeGK(w.shape);
    occ::handle<CapProgress> pg = new CapProgress(ctl);
    BRepOffset_MakeOffset mo;
    mo.Initialize(w.shape, 0.0, 1e-6, BRepOffset_Skin, true, false, GeomAbs_Intersection, false, false);
    for (auto& f : fs) mo.SetOffsetOnFace(f, spec.distanceMm);
    mo.MakeOffsetShape(pg->Start());
    if (pg->broke() || pg->stop()) return stopped(*pg);
    if (!mo.IsDone()) return refuse(KernelOpStatus::Invalid, kE, kF);
    TopoDS_Shape r = oneSolid(mo.Shape()); if (r.IsNull() || !isValid(r)) return refuse(KernelOpStatus::Invalid, kE, kF);
    const double v1 = volumeGK(r), eps = 1e-9 * std::max(1.0, v0);
    if (spec.distanceMm > 0 ? !(v1 > v0 + eps) : !(v1 < v0 - eps)) return refuse(KernelOpStatus::NoEffect, kE, kF);
    TopTools_IndexedMapOfShape rf; TopExp::MapShapes(r, TopAbs_FACE, rf);
    // Every working face keeps its name through Modified (moved faces and their trimmed / extended neighbours); faces OCCT creates are OffsetSide
    // sourced by the face they were generated from.
    std::vector<TopoName> names(rf.Extent()); std::vector<bool> done(rf.Extent(), false);
    for (int i = 1; i <= w.faces.Extent(); ++i) {
      const TopoDS_Shape& f = w.faces(i);
      if (int k = rf.FindIndex(f)) { if (!done[k - 1]) { names[k - 1] = w.faceNames[i - 1]; done[k - 1] = true; } continue; }
      if (mo.IsDeleted(f)) continue;
      // OCCT 8.0.1 (probe): BRepOffset_MakeOffset reports the image of EVERY face (moved or not) through Generated, not Modified. The first image found
      // in the result keeps the face's name; further images are new side faces.
      bool kept = false;
      for (const auto* l : {&mo.Modified(f), &mo.Generated(f)})
        for (auto it = l->begin(); it != l->end(); ++it) if (int k = rf.FindIndex(*it)) if (!done[k - 1] && !kept) { names[k - 1] = w.faceNames[i - 1]; done[k - 1] = true; kept = true; }
    }
    for (int i = 1; i <= w.faces.Extent(); ++i) {
      const Uid src = nameUid(w.faceNames[i - 1]); uint16_t n = 0;
      for (const auto* l : {&mo.Modified(w.faces(i)), &mo.Generated(w.faces(i))})
        for (auto it = l->begin(); it != l->end(); ++it) if (int k = rf.FindIndex(*it)) if (!done[k - 1]) { names[k - 1] = TopoName{feature, Role::OffsetSide, src, n++}; done[k - 1] = true; }
    }
    uint16_t rest = 0; for (int k = 0; k < rf.Extent(); ++k) if (!done[k]) names[k] = TopoName{feature, Role::OffsetSide, {}, rest++};
    makeNamesUnique(names);
    return finish(r, std::move(names));
  } catch (const Standard_Failure&) { return refuse(KernelOpStatus::Invalid, kE, kF); }
  catch (...) { return refuse(KernelOpStatus::Invalid, kE, kF); }
}

// ---------------------------------------------------------------- delete face (OFS-02): remove faces, heal the gap by extending the neighbours
Out deleteFaces(const Input& in, const DeleteFaceSpec& spec, Uid feature, const OpControl& ctl) {
  const char* kE = "These faces cannot be removed cleanly."; const char* kF = "Pick all faces of the feature, for example every face of a hole.";
  try {
    Work w; if (!makeWork(in.shape, in.faces, in.edges, in.faceNames, w)) return refuse(KernelOpStatus::Invalid, kE, kF);
    std::vector<TopoDS_Face> fs; if (!workFaces(in, w, spec.faces, fs)) return refuse(KernelOpStatus::Invalid, "Face reference lost.", "Pick the face again.");
    if ((int)fs.size() >= w.faces.Extent()) return refuse(KernelOpStatus::Invalid, kE, kF);
    occ::handle<CapProgress> pg = new CapProgress(ctl);
    BRepAlgoAPI_Defeaturing df; df.SetShape(w.shape); for (auto& f : fs) df.AddFaceToRemove(f);
    df.SetRunParallel(false); df.SetToFillHistory(true);   // determinism: replay names must not depend on thread order
    df.Build(pg->Start());
    if (pg->broke() || pg->stop()) return stopped(*pg);
    if (!df.IsDone() || df.HasErrors()) return refuse(KernelOpStatus::Invalid, kE, kF);
    TopoDS_Shape r = oneSolid(df.Shape()); if (r.IsNull() || !isValid(r)) return refuse(KernelOpStatus::Invalid, kE, kF);
    TopTools_IndexedMapOfShape rf; TopExp::MapShapes(r, TopAbs_FACE, rf);
    for (auto& f : fs) if (rf.FindIndex(f)) return refuse(KernelOpStatus::NoEffect, kE, kF);
    std::vector<std::pair<TopoDS_Shape, Uid>> gens;
    for (auto& f : fs) { const int k = w.faces.FindIndex(f); gens.push_back({f, k ? nameUid(w.faceNames[k - 1]) : Uid{}}); }
    std::vector<TopoName> names = nameByHistory(df, w, rf, feature, Role::HealFace, gens);
    for (auto& f : fs) { const int k = w.faces.FindIndex(f); if (!k) continue;   // effect check (ADR-041): the deleted names are absent
      for (auto& n : names) if (n == w.faceNames[k - 1]) return refuse(KernelOpStatus::NoEffect, kE, kF); }
    return finish(r, std::move(names));
  } catch (const Standard_Failure&) { return refuse(KernelOpStatus::Invalid, kE, kF); }
  catch (...) { return refuse(KernelOpStatus::Invalid, kE, kF); }
}

}  // namespace chisel::bevel
