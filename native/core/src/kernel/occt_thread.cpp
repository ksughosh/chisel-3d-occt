// SPDX-License-Identifier: LGPL-2.1-only
// W4 T-1 (spec 23 §4, ADR-068): cylinder face detection and the modelled thread cut. Part of the published LGPL source set.
#include <cmath>
#include <optional>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepTools_History.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <Geom2d_Line.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <gp_Dir2d.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Cylinder.hxx>
#include "occt_bevel_guard.h"

namespace chisel::bevel {

// A true cylindrical face: radius, axis (origin = the cylinder's location, dir = its axis), axial range v0..v1 (mm along dir, v0 < v1),
// internal = the material lies outside the radius (the face normal, orientation applied, points to the axis), fullCircle = u-range covers 2*pi.
std::optional<CylFaceInfo> cylinderInfo(const TopoDS_Face& f) {
  try {
    BRepAdaptor_Surface s(f, Standard_True);
    if (s.GetType() != GeomAbs_Cylinder) return std::nullopt;
    const gp_Cylinder cyl = s.Cylinder();
    double u0, u1, v0, v1; BRepTools::UVBounds(f, u0, u1, v0, v1);
    CylFaceInfo c;
    const gp_Ax1 ax = cyl.Axis(); const gp_Pnt o = ax.Location(); const gp_Dir d = ax.Direction();
    c.axis.origin = {o.X(), o.Y(), o.Z()}; c.axis.dir = {d.X(), d.Y(), d.Z()};
    c.radiusMm = cyl.Radius();
    c.v0 = std::min(v0, v1); c.v1 = std::max(v0, v1);
    c.fullCircle = (u1 - u0) >= 2 * M_PI - 1e-7;
    // Orientation-aware normal at the middle of the face vs the radial direction.
    gp_Pnt p; gp_Vec du, dv; s.D1(0.5 * (u0 + u1), 0.5 * (v0 + v1), p, du, dv);
    gp_Vec n = du.Crossed(dv); if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
    const gp_Vec op(o, p); const gp_Vec radial = op - gp_Vec(d) * op.Dot(gp_Vec(d));
    c.internal = n.Dot(radial) < 0;
    if (!(c.radiusMm > 0) || !(c.v1 > c.v0)) return std::nullopt;
    return c;
  } catch (const Standard_Failure&) { return std::nullopt; } catch (...) { return std::nullopt; }
}

TopoDS_Shape helixSweep(const gp_Ax3& ax, double rSpine, double pitch, bool leftHand, double turns, const std::vector<gp_Pnt2d>& rz, bool perTurn, std::string* why) {
  auto bad = [&](const char* w) { if (why) *why = w; return TopoDS_Shape(); };
  try {
    if (!(rSpine > 0) || !(pitch > 0) || !(turns > 0) || rz.size() < 3) return bad("bad input");
    Handle(Geom_CylindricalSurface) cyl = new Geom_CylindricalSurface(ax, rSpine);
    const double su = leftHand ? -2 * M_PI : 2 * M_PI;   // u runs backwards for a left hand; z always advances
    Handle(Geom2d_Line) line = new Geom2d_Line(gp_Pnt2d(0, 0), gp_Dir2d(su, pitch));
    const double perLen = std::hypot(2 * M_PI, pitch), total = perLen * turns;
    BRepBuilderAPI_MakeWire mw;
    if (perTurn) {
      for (double t = 0; t < total - 1e-9; t += perLen) {
        BRepBuilderAPI_MakeEdge me(line, cyl, t, std::min(total, t + perLen)); if (!me.IsDone()) return bad("helix edge");
        mw.Add(me.Edge()); if (!mw.IsDone()) return bad("helix wire");
      }
    } else {
      BRepBuilderAPI_MakeEdge me(line, cyl, 0, total); if (!me.IsDone()) return bad("helix edge");
      mw.Add(me.Edge());
    }
    if (!mw.IsDone()) return bad("helix wire");
    TopoDS_Wire spine = mw.Wire();
    BRepLib::BuildCurves3d(spine);
    const gp_Pnt o = ax.Location(); const gp_Dir x = ax.XDirection(), d = ax.Direction();
    BRepBuilderAPI_MakePolygon poly;
    for (auto& q : rz) poly.Add(o.Translated(gp_Vec(x) * q.X() + gp_Vec(d) * q.Y()));
    poly.Close(); if (!poly.IsDone()) return bad("profile");
    BRepOffsetAPI_MakePipeShell ps(spine);
    ps.SetMode(d);   // constant binormal = the axis: a screw motion (Frenet would twist the profile)
    ps.Add(poly.Wire(), Standard_False, Standard_False);
    ps.Build(); if (!ps.IsDone()) return bad("pipe shell build");
    if (!ps.MakeSolid()) return bad("pipe shell solid");
    return ps.Shape();
  } catch (const Standard_Failure& e) { return bad(e.GetMessageString()); }
  catch (...) { return bad("exception"); }
}

namespace {
// Cancel / time cap polled by OCCT (same contract as occt_modify.cpp's CapProgress).
class ThreadProgress : public Message_ProgressIndicator {
 public:
  explicit ThreadProgress(const OpControl& c) : c_(c), t0_(std::chrono::steady_clock::now()) {}
  bool UserBreak() override {
    if (c_.cancelled && c_.cancelled(c_.user)) { cancelled_ = true; return true; }
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    if (c_.capSeconds > 0 && s > c_.capSeconds) { timedOut_ = true; return true; }
    return false;
  }
  void Show(const Message_ProgressScope&, const bool) override {}
  bool stop() { return UserBreak(); }
  bool broke() const { return cancelled_ || timedOut_; }
  Out stopped() const {
    return cancelled_ ? refuse(KernelOpStatus::Cancelled, "The operation was cancelled.", "Try again.")
                      : refuse(KernelOpStatus::TimedOut, "The operation took too long.", "Try a shorter thread.");
  }
 private:
  OpControl c_; std::chrono::steady_clock::time_point t0_; bool cancelled_ = false, timedOut_ = false;
};

const char* kE = "The thread could not be made.";
const char* kF = "Try a larger pitch or a shorter thread.";
// CHISEL_THREAD_DEBUG=1 prints the failing stage (host diagnostics only).
Out fail(const char* stage) { if (std::getenv("CHISEL_THREAD_DEBUG")) std::fprintf(stderr, "[threadFace] fail at %s\n", stage); return refuse(KernelOpStatus::Invalid, kE, kF); }

// Solid of revolution of the closed (radius, t) polygon about `ax` (t along ax.Direction()).
TopoDS_Shape revolveRZ(const gp_Ax3& ax, const std::vector<gp_Pnt2d>& rz) {
  const gp_Pnt o = ax.Location(); const gp_Dir x = ax.XDirection(), d = ax.Direction();
  BRepBuilderAPI_MakePolygon poly;
  for (auto& q : rz) poly.Add(o.Translated(gp_Vec(x) * q.X() + gp_Vec(d) * q.Y()));
  poly.Close(); if (!poly.IsDone()) return {};
  BRepBuilderAPI_MakeFace mf(poly.Wire(), Standard_True); if (!mf.IsDone()) return {};
  BRepPrimAPI_MakeRevol rv(mf.Face(), ax.Axis(), 2 * M_PI); if (!rv.IsDone()) return {};
  return rv.Shape();
}

// Point at (radius r, axial z) on the +X side of the frame classified against the body: true = outside (a free end).
bool outsideAt(const TopoDS_Shape& body, const gp_Ax3& ax, double r, double z) {
  const gp_Pnt p = ax.Location().Translated(gp_Vec(ax.XDirection()) * r + gp_Vec(ax.Direction()) * z);
  BRepClass3d_SolidClassifier cl(body, p, 1e-7); return cl.State() == TopAbs_OUT;
}
}  // namespace

ThreadProfile threadProfile(const ThreadSpec& spec, double faceR) {
  ThreadProfile t; const double P = spec.pitchMm, H = 0.8660254037844386 * P, c = spec.clearanceMm, k = std::tan(M_PI / 6), wMax = 15 * P / 32;
  t.internal = spec.internal;
  if (!spec.internal) {
    t.rRoot = spec.majorMm / 2 - 17 * H / 24 - c; t.wRoot = P / 12;
    const double rWmax = t.rRoot + (wMax - t.wRoot) / k;   // the flank reaches the minimum crest flat P/16 here
    t.rCrest = std::min(spec.majorMm / 2, rWmax);
    t.rTop = std::max(t.rCrest, faceR) + 0.02 * P;          // overlaps the ring / the air
    const double rk = std::min(t.rTop, rWmax), wk = t.wRoot + (rk - t.rRoot) * k;
    t.poly = {{t.rRoot, -t.wRoot}, {rk, -wk}};
    if (rk < t.rTop) t.poly.push_back({t.rTop, -wMax});
    if (rk < t.rTop) t.poly.push_back({t.rTop, wMax});
    t.poly.push_back({rk, wk}); t.poly.push_back({t.rRoot, t.wRoot});
    t.wHalf = std::min(wMax, t.wRoot + (t.rTop - t.rRoot) * k);
    t.ringIn = t.rCrest; t.ringOut = faceR + 0.05 * P;   // 0.05 P: never coincident with the groove top (0.02 P)
    t.ring = faceR > t.rCrest + 1e-9;
    if (t.ring) {   // the ring removes everything above rCrest: a knee at rCrest = rWmax would put a helical edge ON the ring's inner cylinder
      // (measured 2026-11-14: the cut then splits the rod into 2 solids, nothing removed). Plain trapezoid to rCrest + 0.02 P instead:
      // half-width there = 15P/32 + 0.02P tan30 = 0.480 P < P/2, so the turns never touch; the flanks cross the ring cylinder transversally.
      t.rTop = t.rCrest + 0.02 * P; const double wt = t.wRoot + (t.rTop - t.rRoot) * k;
      t.poly = {{t.rRoot, -t.wRoot}, {t.rTop, -wt}, {t.rTop, wt}, {t.rRoot, t.wRoot}}; t.wHalf = wt;
    }
  } else {
    t.rRoot = spec.majorMm / 2 + c; t.wRoot = P / 16;
    const double rWmax = t.rRoot - (wMax - t.wRoot) / k;
    t.rCrest = std::max(spec.majorMm / 2 - 5 * H / 8, rWmax);
    t.rTop = std::min(t.rCrest, faceR) - 0.02 * P;
    const double rk = std::max(t.rTop, rWmax), wk = t.wRoot + (t.rRoot - rk) * k;
    t.poly = {{t.rRoot, -t.wRoot}, {t.rRoot, t.wRoot}, {rk, wk}};
    if (rk > t.rTop) t.poly.push_back({t.rTop, wMax});
    if (rk > t.rTop) t.poly.push_back({t.rTop, -wMax});
    t.poly.push_back({rk, -wk});
    t.wHalf = std::min(wMax, t.wRoot + (t.rRoot - t.rTop) * k);
    t.ringIn = faceR - 0.05 * P; t.ringOut = t.rCrest; t.ring = faceR < t.rCrest - 1e-9;
  }
  return t;
}

Out thread(const Input& in, const ThreadSpec& spec, Uid feature, const OpControl& ctl) {
  try {
    const double P = spec.pitchMm, c = spec.clearanceMm;
    if (!std::isfinite(P) || !(P > 0) || !std::isfinite(spec.majorMm) || !(spec.majorMm > 0) || !std::isfinite(c) || c < 0 || !std::isfinite(spec.startOffsetMm) ||
        spec.startOffsetMm < 0 || !std::isfinite(spec.lengthMm) || spec.lengthMm < 0 || spec.helix > 1)
      return refuse(KernelOpStatus::Invalid, "Enter a valid size and pitch.", "Check the thread values.");
    int fi = -1; for (size_t i = 0; i < in.faceNames.size(); ++i) if (in.faceNames[i] == spec.face) { fi = (int)i; break; }
    if (fi < 0) return refuse(KernelOpStatus::Invalid, "Reference lost: face.", "Re-pick the face.");
    const TopoDS_Face face = TopoDS::Face(in.faces(fi + 1));
    auto cy = cylinderInfo(face);
    if (!cy || !cy->fullCircle) return refuse(KernelOpStatus::Invalid, "Pick a round face.", "Threads go on a full cylinder.");
    if (cy->internal != spec.internal) return refuse(KernelOpStatus::Invalid, "Reference changed side.", "Re-pick the face.");
    const double R = cy->radiusMm;
    const ThreadProfile tp = threadProfile(spec, R);
    if (!(tp.rRoot > 0) || (!spec.internal && tp.rRoot >= R) || (spec.internal && tp.rRoot <= R))
      return refuse(KernelOpStatus::Invalid, "The thread does not fit this face.", "Check the size against the face diameter.");
    // Extent along the face axis (sg = +1 from v0, -1 from v1 when flipped).
    const double sg = spec.flipStart ? -1 : 1, start = spec.flipStart ? cy->v1 : cy->v0, faceLen = cy->v1 - cy->v0;
    if (spec.startOffsetMm >= faceLen - 1e-9) return refuse(KernelOpStatus::Invalid, "The start offset is past the end of the face.", "Use a smaller offset.");
    const double len = spec.lengthMm > 0 ? std::min(spec.lengthMm, faceLen - spec.startOffsetMm) : faceLen - spec.startOffsetMm;
    const double n = len / P;
    if (n > spec.maxTurns + 1e-9) {
      char e[96]; std::snprintf(e, sizeof e, "Too many turns (%.0f of %d).", std::ceil(n - 1e-9), spec.maxTurns);
      return refuse(KernelOpStatus::Invalid, e, "Shorten the thread or use a larger pitch.");
    }
    glm::dvec3 dd = glm::normalize(cy->axis.dir) * sg;
    const gp_Dir D(dd.x, dd.y, dd.z);
    const glm::dvec3 o = cy->axis.origin + cy->axis.dir * (start + sg * spec.startOffsetMm);   // z = 0 at the thread start, +z into the thread
    const gp_Ax3 base(gp_Pnt(o.x, o.y, o.z), D);
    // Free-end test (spec 23 §4.2 step 3): overrun 1 P at a free end, stop half a groove short at a shoulder.
    const bool freeS = outsideAt(in.shape, base, tp.rRoot, -0.5 * P), freeE = outsideAt(in.shape, base, tp.rRoot, len + 0.5 * P);
    if (spec.leadIn && !freeS) return refuse(KernelOpStatus::Invalid, "A lead-in needs an open end.", "Turn off Lead-in.");
    const double z0 = freeS ? -P : tp.wHalf, z1 = freeE ? len + P : len - tp.wHalf, turnsH = (z1 - z0) / P;
    if (!(turnsH > 1e-6)) return refuse(KernelOpStatus::Invalid, "The thread is too short.", "Make the thread longer.");
    occ::handle<ThreadProgress> pg = new ThreadProgress(ctl);
    const gp_Ax3 hx(base.Location().Translated(gp_Vec(D) * z0), D, base.XDirection());
    NCollection_List<TopoDS_Shape> tools; std::vector<std::pair<TopoDS_Shape, int>> kinds;   // 0 groove, 1 ring, 2 cone
    std::string why;
    // Spec 23 §4.2 step 5: ONE sweep; the spine is n edges of one turn each (helix 0, default) or one edge over all turns (helix 1).
    // Measured on host (2026-11-14): translated per-turn copies of a one-turn sweep work too but cost 3.7x (2312 vs 628 ms, M8x20);
    // the one-edge spine sweeps to a valid solid but BRepAlgoAPI_Cut silently returns the body unchanged (fuzzy 1e-5/1e-3, finer 3-D curve: same).
    {
      TopoDS_Shape s = helixSweep(hx, tp.rRoot, P, spec.leftHand, turnsH, tp.poly, spec.helix == 0, &why); if (s.IsNull()) return fail(why.c_str());
      tools.Append(s); kinds.push_back({s, 0});
    }
    const double ext = 0.02 * P, ra = freeS ? -ext : 0, rb = freeE ? len + ext : len;
    if (tp.ring) {   // large clearance / oversize face: lower the crest to rCrest over the extent
      TopoDS_Shape s = revolveRZ(base, {{tp.ringIn, ra}, {tp.ringOut, ra}, {tp.ringOut, rb}, {tp.ringIn, rb}});
      if (s.IsNull()) return fail("ring"); tools.Append(s); kinds.push_back({s, 1});
    }
    if (spec.leadIn) {   // 45-degree cone at the start end, full thread depth
      std::vector<gp_Pnt2d> tri;
      if (!spec.internal) { const double rm = std::max(R, tp.rCrest) + 0.05 * P; /* not coincident with the groove top */ tri = {{tp.rRoot - ext, -ext}, {rm, -ext}, {rm, rm - tp.rRoot}}; }
      else { const double rm = std::min(R, tp.rCrest) - 0.05 * P; tri = {{rm, -ext}, {tp.rRoot + ext, -ext}, {rm, tp.rRoot + ext - rm}}; }
      TopoDS_Shape s = revolveRZ(base, tri); if (s.IsNull()) return fail("cone"); tools.Append(s); kinds.push_back({s, 2});
    }
    if (pg->stop()) return pg->stopped();   // cancel before the boolean: nothing made
    const double v0 = volumeGK(in.shape);
    BRepAlgoAPI_Cut cut; NCollection_List<TopoDS_Shape> args; args.Append(in.shape);
    cut.SetArguments(args); cut.SetTools(tools); cut.SetRunParallel(false); cut.SetToFillHistory(true);
    cut.Build(pg->Start());
    if (pg->broke() || pg->stop()) return pg->stopped();
    if (!cut.IsDone() || cut.HasErrors()) return fail("cut");
    ShapeUpgrade_UnifySameDomain un(cut.Shape(), Standard_True, Standard_True, Standard_False); un.Build();
    TopoDS_Shape r = un.Shape();
    int solids = 0; for (TopExp_Explorer ex(r, TopAbs_SOLID); ex.More(); ex.Next()) ++solids;
    if (solids != 1) {
      if (std::getenv("CHISEL_THREAD_DEBUG")) for (TopExp_Explorer ex(r, TopAbs_SOLID); ex.More(); ex.Next()) { GProp_GProps g; BRepGProp::VolumeProperties(ex.Current(), g); const gp_Pnt c = g.CentreOfMass(); std::fprintf(stderr, "[threadFace] solid V=%.6g c=(%.3f %.3f %.3f)\n", g.Mass(), c.X(), c.Y(), c.Z()); }
      return fail("solid count");
    } if (!isValid(r)) return fail(isValid(cut.Shape()) ? "invalid after unify" : "invalid cut");
    const double v1 = volumeGK(r);
    if (!(v1 < v0 - 1e-9 * std::max(1.0, v0))) { if (std::getenv("CHISEL_THREAD_DEBUG")) std::fprintf(stderr, "[threadFace] v0 %.6g v1 %.6g warn %d\n", v0, v1, (int)cut.HasWarnings()); fail("no volume decrease");
      if (spec.helix == 1) return refuse(KernelOpStatus::Invalid, "The one-edge helix could not cut this face.", "Use the per-turn helix.");
      return refuse(KernelOpStatus::NoEffect, kE, kF); }
    // Names (ADR-040): surviving input faces keep their names; faces from the tools = {feature, ThreadFace, nameUid(face), k*1000 + i}.
    occ::handle<BRepTools_History> h = new BRepTools_History(*cut.History()); h->Merge(un.History());
    TopTools_IndexedMapOfShape rf; TopExp::MapShapes(r, TopAbs_FACE, rf);
    std::vector<TopoName> names(rf.Extent()); std::vector<bool> done(rf.Extent(), false);
    auto images = [&](const TopoDS_Shape& f, auto&& fn) {
      if (int k = rf.FindIndex(f)) { fn(k); return; }
      if (h->IsRemoved(f)) return;
      for (auto it = h->Modified(f).begin(); it != h->Modified(f).end(); ++it) if (int k = rf.FindIndex(*it)) fn(k);
    };
    for (int i = 1; i <= in.faces.Extent(); ++i) images(in.faces(i), [&](int k) { if (!done[k - 1]) { names[k - 1] = in.faceNames[i - 1]; done[k - 1] = true; } });
    const Uid src = nameUid(spec.face); uint16_t cnt[3] = {0, 0, 0};
    for (auto& [tool, kind] : kinds) {
      TopTools_IndexedMapOfShape tf; TopExp::MapShapes(tool, TopAbs_FACE, tf);
      for (int j = 1; j <= tf.Extent(); ++j)
        images(tf(j), [&](int k) { if (!done[k - 1]) { names[k - 1] = TopoName{feature, Role::ThreadFace, src, (uint16_t)(kind * 1000 + cnt[kind]++)}; done[k - 1] = true; } });
    }
    uint16_t rest = 0; for (int k = 0; k < rf.Extent(); ++k) if (!done[k]) names[k] = TopoName{feature, Role::ThreadFace, src, (uint16_t)(3000 + rest++)};
    makeNamesUnique(names);
    Out out; out.ok = true; out.shape = r; out.faceNames = std::move(names); return out;
  } catch (const Standard_Failure& e) { return fail(e.GetMessageString()); }
  catch (...) { return fail("exception"); }
}

}  // namespace chisel::bevel
