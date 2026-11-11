// SPDX-License-Identifier: LGPL-2.1-only
// F3 two-point bend (G3 spec 5.1..5.5). Part of the published LGPL source set. Step 1: honest refusal stub; the real bend lands function by function.
#include <cmath>
#include <Standard_Failure.hxx>
#include "occt_bevel_guard.h"

#include <algorithm>
#include <vector>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepTools.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRepTools_Modifier.hxx>
#include <BRepTools_NurbsConvertModification.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Ax3.hxx>
#include <gp_Trsf.hxx>

namespace chisel::bevel {
namespace {
// Local frame of a bend (G3 5.3): X = alongDir (anchor A -> B), Y = axisDir (the bend axis), Z = X x Y (the tail moves toward +Z for a positive angle), origin = axisOrigin.
// In it the zone is x in [x0, x1], the reference surface is z = 0 and the neutral surface z = zn; Rn = L / angle is SIGNED (a negative angle bends toward -Z).
struct Frame {
  glm::dvec3 o{0}, x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1}; double x0 = 0, x1 = 0, zn = 0, ang = 0, L = 0, Rn = 0;
  gp_Trsf toLocal, toGlobal;
};
// false when a direction is degenerate (zero, not finite, not perpendicular) or the zone / angle is not finite or has no length.
[[maybe_unused]] bool makeFrame(const BendSpec& s, Frame& f) {
  if (!finiteV(s.axisOrigin) || !finiteV(s.axisDir) || !finiteV(s.alongDir)) return false;
  if (!std::isfinite(s.zoneStart) || !std::isfinite(s.zoneEnd) || !std::isfinite(s.neutralOffsetMm) || !std::isfinite(s.angleRad)) return false;
  if (glm::length(s.axisDir) < 1e-12 || glm::length(s.alongDir) < 1e-12) return false;
  f.y = glm::normalize(s.axisDir); f.x = glm::normalize(s.alongDir);
  if (std::abs(glm::dot(f.x, f.y)) > 1e-6) return false;
  f.z = glm::cross(f.x, f.y); f.o = s.axisOrigin; f.x0 = s.zoneStart; f.x1 = s.zoneEnd; f.zn = s.neutralOffsetMm; f.ang = s.angleRad; f.L = f.x1 - f.x0;
  if (f.L < 1e-3) return false;
  f.Rn = std::abs(f.ang) < 1e-12 ? 0 : f.L / f.ang;
  f.toLocal.SetTransformation(gp_Ax3(gp_Pnt(f.o.x, f.o.y, f.o.z), gp_Dir(f.z.x, f.z.y, f.z.z), gp_Dir(f.x.x, f.x.y, f.x.z)));
  f.toGlobal = f.toLocal.Inverted(); return true;
}
// The ideal bend map in the local frame: x < x0 identity, zone = rotation about the centre (x0, zn + Rn) with the neutral surface length preserved, x > x1 rigid.
[[maybe_unused]] glm::dvec3 bendMap(const Frame& f, glm::dvec3 p) {
  if (f.Rn == 0 || p.x <= f.x0) return p;
  double s = std::min(p.x, f.x1) - f.x0, phi = f.ang * s / f.L, d = p.z - f.zn, h = std::sin(0.5 * phi);
  glm::dvec3 q(f.x0 + (f.Rn - d) * std::sin(phi), p.y, f.zn + d * std::cos(phi) + 2 * f.Rn * h * h);   // 2 Rn sin^2(phi/2) = Rn (1 - cos phi), stable for small phi
  if (p.x > f.x1) q += glm::dvec3(std::cos(f.ang), 0, std::sin(f.ang)) * (p.x - f.x1);
  return q;
}
// Knot parameters for a parametric direction whose local x runs from xa to xb over [t0, t1]. A straight (linear, non rational) direction gets the exact anchors x0 / x1 plus N uniform
// cells across the zone; a curved one gets N uniform parameter cells. A direction whose x range does not overlap the zone gets none.
std::vector<double> zoneKnots(const Frame& f, double xa, double xb, double t0, double t1, bool straight, int N) {
  double lo = std::min(xa, xb), hi = std::max(xa, xb); std::vector<double> out;
  if (hi - lo < 1e-6 || hi <= f.x0 + 1e-9 || lo >= f.x1 - 1e-9) return out;
  if (!straight) { for (int i = 1; i < N; ++i) out.push_back(t0 + (t1 - t0) * i / N); return out; }
  std::vector<double> xs = {f.x0, f.x1}; for (int i = 1; i < N; ++i) xs.push_back(f.x0 + f.L * i / N);
  for (double x : xs) if (x > lo + 1e-9 && x < hi - 1e-9) out.push_back(t0 + (x - xa) / (xb - xa) * (t1 - t0));
  std::sort(out.begin(), out.end()); out.erase(std::unique(out.begin(), out.end(), [](double a, double b) { return std::abs(a - b) < 1e-12; }), out.end()); return out;
}

// Pole-mapping modification (spike C8-R-BEND, spec 5.5a): NURBS convert (base class), insert knots, replace every pole by bendMap(pole), keep the ORIGINAL p-curves (recomputed ones make
// the solid invalid, QA: 36.7 mm edge tolerance), move vertices by the same map. Works in the LOCAL frame (the caller transforms the shape in and out). Degree 1 stays degree 1.
class BendMod : public BRepTools_NurbsConvertModification {
 public:
  BendMod(const Frame& f, int N) : f_(f), N_(N) {}
  bool NewSurface(const TopoDS_Face& F, occ::handle<Geom_Surface>& S, TopLoc_Location& L, double& Tol, bool& RW, bool& RF) override {
    if (!BRepTools_NurbsConvertModification::NewSurface(F, S, L, Tol, RW, RF)) return false;
    auto bs = occ::down_cast<Geom_BSplineSurface>(S); if (bs.IsNull()) return true;
    auto g = occ::down_cast<Geom_BSplineSurface>(bs->Copy());
    if (!L.IsIdentity()) { g->Transform(L.Transformation()); L = TopLoc_Location(); }   // bake the location into the poles before the map (spike problem 3)
    double u0, u1, v0, v1; g->Bounds(u0, u1, v0, v1); auto orig = occ::down_cast<Geom_BSplineSurface>(g->Copy()); int nk[2] = {0, 0};
    for (int dir = 0; dir < 2; ++dir) {
      bool uD = dir == 0; int I = uD ? g->NbUPoles() : g->NbVPoles(), J = uD ? g->NbVPoles() : g->NbUPoles(); double span = 0, xa = 0, xb = 0;
      for (int j = 1; j <= J; ++j) { double a = g->Pole(uD ? 1 : j, uD ? j : 1).X(), c = g->Pole(uD ? I : j, uD ? j : I).X(); if (std::abs(c - a) > span) { span = std::abs(c - a); xa = a; xb = c; } }
      double lo = 1e300, hi = -1e300;   // x extent ALONG this direction: the pole line with the largest range decides (a direction across the width has constant x and gets no knots)
      for (int j = 1; j <= J; ++j) { double l0 = 1e300, h0 = -1e300; for (int i = 1; i <= I; ++i) { double x = g->Pole(uD ? i : j, uD ? j : i).X(); l0 = std::min(l0, x); h0 = std::max(h0, x); } if (h0 - l0 > hi - lo || lo > hi) { lo = l0; hi = h0; } }
      bool straight = span > 1e-6 && !(uD ? g->IsURational() : g->IsVRational()); double t0 = uD ? u0 : v0, t1 = uD ? u1 : v1;
      if (!straight) { xa = lo; xb = hi; }
      auto ks = zoneKnots(f_, xa, xb, t0, t1, straight, N_); nk[dir] = (int)ks.size();
      for (double k : ks) { if (uD) g->InsertUKnot(k, 1, 1e-9, false); else g->InsertVKnot(k, 1, 1e-9, false); }
    }
    for (int i = 1; i <= g->NbUPoles(); ++i) for (int j = 1; j <= g->NbVPoles(); ++j) { auto q = bendMap(f_, {g->Pole(i, j).X(), g->Pole(i, j).Y(), g->Pole(i, j).Z()}); g->SetPole(i, j, gp_Pnt(q.x, q.y, q.z)); }
    // devSurf (spec 5.5a 1): max |bendMap(S(u,v)) - B(u,v)| on a grid of 33 samples per cell span (9 across a direction that gets no knots), the spike's metric (sampled: a lower bound)
    int su = nk[0] ? 33 * N_ : 9, sv = nk[1] ? 33 * N_ : 9;
    for (int a = 0; a <= su; ++a) for (int c = 0; c <= sv; ++c) {
      double u = u0 + (u1 - u0) * a / su, v = v0 + (v1 - v0) * c / sv; gp_Pnt p = orig->Value(u, v), q = g->Value(u, v); auto w = bendMap(f_, {p.X(), p.Y(), p.Z()});
      dev = std::max(dev, glm::length(w - glm::dvec3(q.X(), q.Y(), q.Z())));
    }
    S = g; return true;
  }
  bool NewCurve(const TopoDS_Edge& E, occ::handle<Geom_Curve>& C, TopLoc_Location& L, double& Tol) override {
    if (!BRepTools_NurbsConvertModification::NewCurve(E, C, L, Tol)) return false;
    auto bc = occ::down_cast<Geom_BSplineCurve>(C); if (bc.IsNull()) return true;
    auto c = occ::down_cast<Geom_BSplineCurve>(bc->Copy()); if (!L.IsIdentity()) { c->Transform(L.Transformation()); L = TopLoc_Location(); }
    double t0 = c->FirstParameter(), t1 = c->LastParameter(), lo = 1e300, hi = -1e300; for (int i = 1; i <= c->NbPoles(); ++i) { lo = std::min(lo, c->Pole(i).X()); hi = std::max(hi, c->Pole(i).X()); }
    double xa = c->Pole(1).X(), xb = c->Pole(c->NbPoles()).X(); bool straight = !c->IsRational() && std::abs(xb - xa) > 1e-6; if (!straight) { xa = lo; xb = hi; }
    auto orig = occ::down_cast<Geom_BSplineCurve>(c->Copy()); auto ks = zoneKnots(f_, xa, xb, t0, t1, straight, N_); for (double k : ks) c->InsertKnot(k, 1, 1e-9, false);
    for (int i = 1; i <= c->NbPoles(); ++i) { auto q = bendMap(f_, {c->Pole(i).X(), c->Pole(i).Y(), c->Pole(i).Z()}); c->SetPole(i, gp_Pnt(q.x, q.y, q.z)); }
    int ns = ks.empty() ? 4 : 4 * N_; for (int a = 0; a <= ns; ++a) { double t = t0 + (t1 - t0) * a / ns; gp_Pnt p = orig->Value(t), q = c->Value(t); auto w = bendMap(f_, {p.X(), p.Y(), p.Z()}); dev = std::max(dev, glm::length(w - glm::dvec3(q.X(), q.Y(), q.Z()))); }
    C = c; return true;
  }
  // The PARAMETRISATIONS are untouched by a pole map (only poles move): the original p-curves stay exact, so reuse them instead of the base class's projection.
  bool NewCurve2d(const TopoDS_Edge& E, const TopoDS_Face& F, const TopoDS_Edge&, const TopoDS_Face&, occ::handle<Geom2d_Curve>& C, double& Tol) override {
    double a, b; occ::handle<Geom2d_Curve> pc = BRep_Tool::CurveOnSurface(E, F, a, b); if (pc.IsNull()) return false; C = pc; Tol = BRep_Tool::Tolerance(E); return true;
  }
  bool NewPoint(const TopoDS_Vertex& V, gp_Pnt& Pt, double& Tol) override {
    // the base class returns false and does NOT fill Pt (it stayed at the origin: every vertex collapsed, the solid was invalid and meshed to nothing): read the vertex ourselves, keep its tolerance
    Pt = BRep_Tool::Pnt(V); Tol = BRep_Tool::Tolerance(V); auto q = bendMap(f_, {Pt.X(), Pt.Y(), Pt.Z()}); Pt = gp_Pnt(q.x, q.y, q.z); return true;
  }
 double dev = 0;   // RAW max deviation of every bent face (and 4 samples per cell of every edge) from the ideal map, mm
 private:
  Frame f_; int N_;
};

// NURBS route: shape (global) -> local frame -> pole map with N cells -> back to global. `names` = the input face names mapped one-to-one through ModifiedShape (G3 5.5 namesOneToOne).
// Returns a null shape when the modifier fails; the caller validates.
TopoDS_Shape nurbsBend(const Input& in, const Frame& f, int N, std::vector<TopoName>& names, double& dev) {
  BRepBuilderAPI_Transform tin(in.shape, f.toLocal, true); if (!tin.IsDone()) return {};
  TopoDS_Shape local = tin.Shape(); occ::handle<BendMod> mod = new BendMod(f, N);
  BRepTools_Modifier md(local); md.Perform(mod); if (!md.IsDone()) return {}; dev = mod->dev;
  TopoDS_Shape out = md.ModifiedShape(local); if (out.IsNull()) return {};
  BRepBuilderAPI_Transform tout(out, f.toGlobal, true); if (!tout.IsDone()) return {};
  TopoDS_Shape res = tout.Shape(); TopTools_IndexedMapOfShape rf; TopExp::MapShapes(res, TopAbs_FACE, rf);
  if (rf.Extent() != in.faces.Extent() || in.faces.Extent() != (int)in.faceNames.size()) return {};   // one-to-one: no face split or merged
  names.assign(rf.Extent(), TopoName{}); std::vector<bool> got(rf.Extent(), false);
  for (int i = 1; i <= in.faces.Extent(); ++i) {   // input face -> local copy -> bent face -> global copy, every step through the History of the operator
    int k = rf.FindIndex(tout.ModifiedShape(md.ModifiedShape(tin.ModifiedShape(in.faces(i))))); if (k < 1 || got[k - 1]) return {}; names[k - 1] = in.faceNames[i - 1]; got[k - 1] = true;
  }
  BRepTools::Clean(res);   // drop any triangulation carried over from the input: the poles moved, a stale mesh would be wrong
  return res;
}

constexpr double kMaxAngleRad = 170.0 * M_PI / 180.0;   // HYPOTHESIS: the spike measured up to 170 deg; larger (and 360) is refused, the tail would meet the body
constexpr double kZeroAngleRad = 1e-9;                  // below this the bend is the identity (exact)
const char* kE = "The bend could not be made.";

// Guards before any geometry is touched (G3 5.5 guardsAndRefusals). On refusal `no` carries the text and the function returns false.
bool preflight(const Input& in, const BendSpec& s, Frame& f, Out& no) {
  if (!makeFrame(s, f)) { no = refuse(kE, "Move the bend points apart."); return false; }
  if (std::abs(s.angleRad) > kMaxAngleRad) { no = refuse("The bend angle is too large.", "Use an angle of at most 170 degrees."); return false; }
  if (!isValid(in.shape)) { no = refuse("This body has gaps in its surface and cannot be bent.", "Pick a solid body."); return false; }   // S11
  BRepBuilderAPI_Transform t(in.shape, f.toLocal, false); Bnd_Box b; BRepBndLib::Add(t.Shape(), b); if (b.IsVoid()) { no = refuse(kE, "Pick a solid body."); return false; }
  double x0, y0, z0, x1, y1, z1; b.Get(x0, y0, z0, x1, y1, z1);
  if (x1 <= f.x0 + 1e-6 || x0 >= f.x1 - 1e-6) { no = refuse("The bend zone does not cross the body.", "Move the bend points onto the body."); return false; }
  return true;
}

// Validity + volume guards after the pole map (G3 5.5a rules). A body with a hole in the zone is INVALID after the map (spike problem 2): SameParameter(1e-3, force) + UpdateTolerances repairs
// it; if the shape is still invalid, or the volume is not finite / positive / within kVolTol of the input (a bend preserves volume; chord sag removes ~1e-4..1e-3), it is refused.
// Start N (G3 5.5a 3, HYPOTHESIS: the zone-length scaling is not measured): clamp(ceil(32 sqrt(theta/90deg) L/40mm), 16, 128).
int startCells(const Frame& f) { return std::clamp((int)std::ceil(32.0 * std::sqrt(std::abs(f.ang) / (M_PI / 2)) * f.L / 40.0), 16, 128); }
double maxEdgeTol(const TopoDS_Shape& r) { double t = 0; for (TopExp_Explorer ex(r, TopAbs_EDGE); ex.More(); ex.Next()) t = std::max(t, BRep_Tool::Tolerance(TopoDS::Edge(ex.Current()))); return t; }
// Gauss-Kronrod volume at 1e-6 relative (spike: 2.7 ms). The shared volumeGK() asks for 1e-9, which takes 6..54 s on a bent chamfered body (measured), so the bend uses its own tolerance.
double volumeBent(const TopoDS_Shape& s) { GProp_GProps g; BRepGProp::VolumePropertiesGK(s, g, 1e-6); return g.Mass(); }
constexpr double kVolTol = 0.02;   // HYPOTHESIS: relative volume change that no honest bend (<= 170 deg, N >= 16) reaches; a sign-flipped map gives a negative or doubled volume
bool guardResult(TopoDS_Shape& r, double v0, double& v1, bool& repaired) {
  repaired = false;
  if (!isValid(r)) { BRepLib::SameParameter(r, 1e-3, true); BRepLib::UpdateTolerances(r); repaired = true; if (!isValid(r)) return false; }
  v1 = volumeBent(r);
  return std::isfinite(v1) && v1 > 0 && std::abs(v1 - v0) <= kVolTol * v0;
}
}  // namespace

Out bend(const Input& in, const BendSpec& spec, Uid feature, BendReport* rep) {
  (void)feature; if (rep) *rep = BendReport{};
  try {
    Frame f; Out no; if (!preflight(in, spec, f, no)) return no;
    if (std::abs(spec.angleRad) < kZeroAngleRad) {   // a straight bend is the identity: a deep copy with the same names (exact)
      BRepBuilderAPI_Copy cp(in.shape, true); Out o; o.ok = true; o.shape = cp.Shape(); o.faceNames = in.faceNames;
      if (rep) { rep->exact = true; rep->boundKnown = true; rep->volumeMm3 = volumeBent(o.shape); rep->cells = 0; rep->degree = 1; } return o;
    }
    double v0 = volumeBent(in.shape); if (!(v0 > 0)) return refuse("This body has gaps in its surface and cannot be bent.", "Pick a solid body.");
    int N = spec.cells > 0 ? std::min(spec.cells, 128) : startCells(f); std::vector<TopoName> names; TopoDS_Shape r; double dev = 0, v1 = 0, tol = 0; bool repaired = false;
    for (;;) {   // G3 5.5a 3: double N until the bound (raw devSurf + maxEdgeTol) reaches the target or N = 128; the bend is still made when 128 does not reach it
      r = nurbsBend(in, f, N, names, dev); if (r.IsNull()) return refuse(kE, "Try a smaller angle.");
      if (!guardResult(r, v0, v1, repaired)) return refuse(kE, "Try a smaller angle.");
      tol = maxEdgeTol(r); if (spec.cells > 0 || std::max(dev, tol) <= spec.targetDeviationMm || N >= 128) break;
      N = std::min(128, N * 2);
    }
    if (rep) { rep->exact = false; rep->boundKnown = true; rep->devSurfMm = dev; rep->maxEdgeTolMm = tol; rep->volumeMm3 = v1; rep->cells = N; rep->degree = 1; }
    Out o; o.ok = true; o.shape = r; o.faceNames = std::move(names); return o;
  } catch (const Standard_Failure&) { return refuse(kE, "Try a smaller angle."); } catch (...) { return refuse(kE, "Try a smaller angle."); }
}

}  // namespace chisel::bevel
