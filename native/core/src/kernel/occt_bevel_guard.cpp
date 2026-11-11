// SPDX-License-Identifier: LGPL-2.1-only
// KERN-5 guard helpers (G2 spec 4.1b/4.1c). See occt_bevel_guard.h. Part of the published LGPL source set.
#include "occt_bevel_guard.h"
#include <algorithm>
#include <array>
#include <set>
#include <cmath>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepGProp_Face.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Pnt2d.hxx>
#include <BRepTools_History.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>

namespace chisel::bevel {

Out refuse(const char* error, const char* fix) { Out o; o.ok = false; o.error = error; o.fix = fix; return o; }

bool finiteV(const glm::dvec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

double volumeGK(const TopoDS_Shape& s) {
  GProp_GProps g; BRepGProp::VolumePropertiesGK(s, g, 1e-9); return g.Mass();   // the default call is off by up to 1.6 % on bent bodies (QA)
}

bool isValid(const TopoDS_Shape& s) { return !s.IsNull() && BRepCheck_Analyzer(s).IsValid(); }

int faceCount(const TopoDS_Shape& s) { TopTools_IndexedMapOfShape m; TopExp::MapShapes(s, TopAbs_FACE, m); return m.Extent(); }

bool sameEdgeName(const EdgeName& a, const EdgeName& b) { return a.a == b.a && a.b == b.b && a.tie == b.tie; }

std::vector<EdgeName> edgeNamesOf(const TopTools_IndexedMapOfShape& faces, const TopTools_IndexedMapOfShape& edges, const std::vector<TopoName>& faceNames) {
  std::vector<EdgeName> out(edges.Extent()); std::vector<std::array<long long, 3>> key(edges.Extent());
  std::vector<std::vector<int>> adj(edges.Extent() + 1);
  for (int f = 1; f <= faces.Extent(); ++f) {
    TopTools_IndexedMapOfShape fe; TopExp::MapShapes(faces(f), TopAbs_EDGE, fe);
    for (int k = 1; k <= fe.Extent(); ++k) if (int ei = edges.FindIndex(fe(k))) if (std::find(adj[ei].begin(), adj[ei].end(), f) == adj[ei].end()) adj[ei].push_back(f);
  }
  for (int e = 1; e <= edges.Extent(); ++e) {
    BRepAdaptor_Curve c(TopoDS::Edge(edges(e))); gp_Pnt m = c.Value(0.5 * (c.FirstParameter() + c.LastParameter()));
    key[e - 1] = {llround(m.X() * 1e6), llround(m.Y() * 1e6), llround(m.Z() * 1e6)};
    std::vector<TopoName> ns; for (int f : adj[e]) if ((size_t)f - 1 < faceNames.size()) ns.push_back(faceNames[f - 1]);
    std::sort(ns.begin(), ns.end()); EdgeName n; if (!ns.empty()) { n.a = ns.front(); n.b = ns.back(); } out[e - 1] = n;
  }
  std::vector<int> order(edges.Extent()); for (int i = 0; i < edges.Extent(); ++i) order[i] = i;
  std::stable_sort(order.begin(), order.end(), [&](int x, int y) { return key[x] < key[y]; });
  std::vector<std::pair<std::pair<TopoName, TopoName>, int>> seen;
  for (int i : order) {
    auto p = std::make_pair(out[i].a, out[i].b); int cnt = 0;
    for (auto& s : seen) if (s.first == p) { cnt = ++s.second; break; }
    if (cnt == 0) seen.push_back({p, 0});
    out[i].tie = (uint8_t)std::min(cnt, 255);
  }
  return out;
}

Uid edgeUid(const EdgeName& e) {
  auto fnv = [](uint64_t h, const std::string& s) { for (unsigned char c : s) { h ^= c; h *= 1099511628211ull; } return h; };
  std::string a = e.a.str() + "|" + e.a.source.hex(), b = e.b.str() + "|" + e.b.source.hex();
  Uid u; u.hi = fnv(1469598103934665603ull, a + "/" + b); u.lo = fnv(0x9E3779B97F4A7C15ull, b + "/" + a + "#" + std::to_string(e.tie)); return u;
}

void makeNamesUnique(std::vector<TopoName>& names) {
  std::set<TopoName> seen;
  for (auto& n : names) { while (!seen.insert(n).second) ++n.split; }
}

// Outward normal of face `f` at the midpoint of edge `e`, and the edge tangent as traversed in the face wire (face on the left seen from outside).
static bool normalAtEdge(const TopoDS_Face& f, const TopoDS_Edge& e, gp_Vec& n, gp_Vec& t, gp_Pnt& p) {
  for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next()) {
    if (!ex.Current().IsSame(e)) continue;
    TopoDS_Edge ee = TopoDS::Edge(ex.Current()); double a, b; Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(ee, f, a, b); if (pc.IsNull()) return false;
    gp_Pnt2d uv = pc->Value(0.5 * (a + b)); BRepGProp_Face gf(f); gp_Vec nn; gf.Normal(uv.X(), uv.Y(), p, nn);
    if (nn.Magnitude() < 1e-14) return false; n = nn.Normalized();
    BRepAdaptor_Curve c(ee); gp_Pnt q; gp_Vec d; c.D1(0.5 * (c.FirstParameter() + c.LastParameter()), q, d);
    if (d.Magnitude() < 1e-14) return false; t = d.Normalized(); if (ee.Orientation() == TopAbs_REVERSED) t.Reverse();
    return true;
  }
  return false;
}

Convexity edgeConvexity(const TopTools_IndexedDataMapOfShapeListOfShape& anc, const TopoDS_Edge& e) {
  int k = anc.FindIndex(e); if (!k) return Convexity::Other;
  const auto& l = anc(k); if (l.Size() != 2 || l.First().IsSame(l.Last()) || BRep_Tool::Degenerated(e)) return Convexity::Other;
  gp_Vec n1, t1, n2, t2; gp_Pnt p1, p2;
  if (!normalAtEdge(TopoDS::Face(l.First()), e, n1, t1, p1) || !normalAtEdge(TopoDS::Face(l.Last()), e, n2, t2, p2)) return Convexity::Other;
  if (n1.Crossed(n2).Magnitude() < 1e-6 && n1.Dot(n2) > 0) return Convexity::Smooth;   // coplanar / tangent faces: not a real edge
  gp_Vec in1 = n1.Crossed(t1);   // direction into face 1, away from the edge (face on the left of the traversal, seen from outside)
  double s = in1.Dot(n2);        // convex: face 1 lies behind the plane of face 2 (s < 0); concave: in front (s > 0)
  if (std::abs(s) < 1e-9) return Convexity::Other;
  return s < 0 ? Convexity::Convex : Convexity::Concave;
}

// First successor of `s` through (copy, unify history); null when it vanished or was split in several.
static TopoDS_Shape successor(BRepBuilderAPI_Copy& cp, const Handle(BRepTools_History)& h, const TopoDS_Shape& in) {
  TopoDS_Shape c = cp.ModifiedShape(in); if (c.IsNull()) return c;
  if (h.IsNull()) return c;
  if (h->IsRemoved(c)) return TopoDS_Shape();
  const auto& m = h->Modified(c); if (m.IsEmpty()) return c;
  return m.Size() == 1 ? m.First() : TopoDS_Shape();
}

bool makeWork(const TopoDS_Shape& shape, const TopTools_IndexedMapOfShape& inFaces, const TopTools_IndexedMapOfShape& inEdges,
              const std::vector<TopoName>& inFaceNames, Work& w) {
  BRepBuilderAPI_Copy cp(shape, true);   // deep copy: the input body is never touched (G2 4.1b 6)
  ShapeUpgrade_UnifySameDomain un(cp.Shape(), true, true, true); un.Build();
  w.shape = un.Shape(); if (w.shape.IsNull()) return false;
  TopExp::MapShapes(w.shape, TopAbs_FACE, w.faces); TopExp::MapShapes(w.shape, TopAbs_EDGE, w.edges);
  w.faceNames.assign(w.faces.Extent(), TopoName{}); std::vector<bool> named(w.faces.Extent(), false);
  w.faceOfInput.assign(inFaces.Extent(), -1); w.edgeOfInput.assign(inEdges.Extent(), -1);
  for (int i = 1; i <= inFaces.Extent(); ++i) {
    TopoDS_Shape s = successor(cp, un.History(), inFaces(i)); int k = s.IsNull() ? 0 : w.faces.FindIndex(s); if (!k || (size_t)i > inFaceNames.size()) continue;
    w.faceOfInput[i - 1] = k - 1;
    // merged face: the smallest predecessor name (deterministic, stable on replay; G2 4.1c)
    if (!named[k - 1] || inFaceNames[i - 1] < w.faceNames[k - 1]) { w.faceNames[k - 1] = inFaceNames[i - 1]; named[k - 1] = true; }
  }
  for (int i = 1; i <= inEdges.Extent(); ++i) {
    TopoDS_Shape s = successor(cp, un.History(), inEdges(i)); int k = s.IsNull() ? 0 : w.edges.FindIndex(s); if (k) w.edgeOfInput[i - 1] = k - 1;
  }
  w.edgeNames = edgeNamesOf(w.faces, w.edges, w.faceNames);
  return true;
}

}  // namespace chisel::bevel
