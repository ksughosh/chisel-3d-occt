// SPDX-License-Identifier: LGPL-2.1-only
// L1 OcctKernel (OCCT 8.0.1, shared libraries). No OCCT type crosses this file (spec 03 4.7).
#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <exception>
#include <unordered_map>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BOPAlgo_Operation.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <Bnd_Box.hxx>
#include <Standard_Failure.hxx>
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
  KernelCaps caps() const override { KernelCaps c; c.booleans = true; c.intersect = true; c.exactBrep = true; c.namesThroughBooleans = true; c.copy = true; return c; }   // Intersect shipped in KERN-2 (BRepAlgoAPI_Common); toFace lands with M4-A

  struct Built { TopoDS_Face face; std::vector<std::pair<TopoDS_Edge, Uid>> edges; bool ok = false; std::string err; };

  Built buildFace(const Profile2D& pr) {
    Built b; const PlaneFrame& f = pr.plane;
    gp_Pln pln(gp_Pnt(f.origin.x, f.origin.y, f.origin.z), D(f.normal));
    std::vector<TopoDS_Wire> wires;
    for (auto& loop : pr.loops) {
      BRepBuilderAPI_MakeWire mw;
      for (auto& e : loop) {
        TopoDS_Edge edge;
        if (e.kind == LoopEdge::Line) {
          if (glm::distance(e.a, e.b) < 1e-9) continue;
          edge = BRepBuilderAPI_MakeEdge(P(f, e.a), P(f, e.b));
        } else if (e.kind == LoopEdge::Circle) {
          gp_Ax2 ax(P(f, e.c), D(f.normal), D(f.xAxis));
          edge = BRepBuilderAPI_MakeEdge(gp_Circ(ax, e.r));
          if (!e.ccw) edge.Reverse();
        } else {
          gp_Ax2 ax(P(f, e.c), D(f.normal), D(f.xAxis));
          GC_MakeArcOfCircle arc(gp_Circ(ax, e.r), P(f, e.a), P(f, e.b), e.ccw);
          if (!arc.IsDone()) { b.err = "An arc in the profile could not be built."; return b; }
          edge = BRepBuilderAPI_MakeEdge(arc.Value());
        }
        mw.Add(edge); b.edges.emplace_back(edge, e.source);
      }
      if (!mw.IsDone()) { b.err = "The profile loop is not closed."; return b; }
      wires.push_back(mw.Wire());
    }
    if (wires.empty()) { b.err = "The profile is empty."; return b; }
    BRepBuilderAPI_MakeFace mf(pln, wires[0], true);
    for (size_t i = 1; i < wires.size(); ++i) mf.Add(wires[i]);
    if (!mf.IsDone()) { b.err = "The profile cannot form a face."; return b; }
    ShapeFix_Face fix(mf.Face()); fix.Perform();
    b.face = TopoDS::Face(fix.Face()); b.ok = true;
    // ShapeFix_Face may rebuild the wires, so Generated(edge) on the ORIGINAL edges found only some side faces and left the rest with the same
    // default name (two faces, one name). Re-key on the edges of the fixed face, matched to their source entity by edge midpoint (M2-C).
    {
      TopTools_IndexedMapOfShape fe; TopExp::MapShapes(b.face, TopAbs_EDGE, fe);
      auto mid = [](const TopoDS_Edge& e) { BRepAdaptor_Curve c(e); return c.Value((c.FirstParameter() + c.LastParameter()) / 2); };
      std::vector<std::pair<TopoDS_Edge, Uid>> remap;
      for (int i = 1; i <= fe.Extent(); ++i) {
        TopoDS_Edge e = TopoDS::Edge(fe(i)); gp_Pnt p = mid(e); double best = 1e300; Uid src;
        for (auto& [oe, u] : b.edges) { double d = p.Distance(mid(oe)); if (d < best) { best = d; src = u; } }
        if (best < 1e-5) remap.emplace_back(e, src);
      }
      if (remap.size() == (size_t)fe.Extent() && !remap.empty()) b.edges = std::move(remap);
    }
    return b;
  }

  KernelResult finishBody(TopoDS_Shape solid, NameMap names, Uid feature) {
    KernelResult r;
    (void)feature;
    Body b; b.shape = solid; b.names = std::move(names);
    TopExp::MapShapes(solid, TopAbs_FACE, b.faces);
    std::lock_guard<std::mutex> lk(m_);
    r.ok = true; r.body.id = ++next_; r.names = b.names; bodies_.emplace(r.body.id, std::move(b));
    return r;
  }

  // NaN/inf survive glm::normalize and gp_Dir (OCCT's zero-magnitude test is false for NaN), so the adapter rejects them before building.
  static bool finite(V3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
  static bool finiteProfile(const Profile2D& pr) {
    const PlaneFrame& f = pr.plane;
    if (!finite(f.origin) || !finite(f.normal) || !finite(f.xAxis) || !finite(f.yAxis) || glm::length(f.normal) < 1e-12 || glm::length(f.xAxis) < 1e-12) return false;
    for (auto& l : pr.loops) for (auto& e : l)
      if (!std::isfinite(e.a.x) || !std::isfinite(e.a.y) || !std::isfinite(e.b.x) || !std::isfinite(e.b.y) || !std::isfinite(e.c.x) || !std::isfinite(e.c.y) || !std::isfinite(e.r)) return false;
    return true;
  }

  KernelResult fail(const std::string& e, const std::string& f) { KernelResult r; r.ok = false; r.error = e; r.fix = f; return r; }

  KernelResult extrudeImpl(const Profile2D& pr, const ExtrudeSpec& s, Uid feature) {
    if (pr.loops.empty() || pr.areaMm2 <= 0) return fail("The profile is empty.", "Select a closed region.");
    if (!finiteProfile(pr) || !std::isfinite(s.distanceMm)) return fail("The profile has invalid geometry.", "Check the sketch profile.");
    if (std::abs(s.distanceMm) < 1e-12) return fail("Distance is 0.", "Enter a non-zero distance.");
    Built b = buildFace(pr); if (!b.ok) return fail(b.err, "Check the sketch profile.");
    V3 n = pr.plane.normal * (s.flip ? -1.0 : 1.0); double d = std::abs(s.distanceMm); if (s.distanceMm < 0) n = -n;
    TopoDS_Shape face = b.face;
    if (s.symmetric) { gp_Trsf t; t.SetTranslation(gp_Vec(-n.x * d / 2, -n.y * d / 2, -n.z * d / 2)); face = BRepBuilderAPI_Transform_Local(face, t); }
    BRepPrimAPI_MakePrism prism(face, gp_Vec(n.x * d, n.y * d, n.z * d));
    if (!prism.IsDone()) return fail("The extrusion failed.", "Change the distance or the profile.");
    NameMap names; TopoDS_Shape solid = prism.Shape();
    TopTools_IndexedMapOfShape faces; TopExp::MapShapes(solid, TopAbs_FACE, faces);
    names.faces.assign(faces.Extent(), TopoName{feature, Role::Side, {}, 0});
    auto idx = [&](const TopoDS_Shape& s2) { return faces.FindIndex(s2); };
    if (int i = idx(prism.FirstShape())) names.faces[i - 1] = TopoName{feature, Role::CapStart, {}, 0};
    if (int i = idx(prism.LastShape())) names.faces[i - 1] = TopoName{feature, Role::CapEnd, {}, 0};
    std::map<Uid, int> splitN;
    for (auto& [edge, src] : b.edges) {
      TopTools_ListOfShape gen = prism.Generated(edge);
      for (auto it = gen.begin(); it != gen.end(); ++it) if (int i = idx(*it)) names.faces[i - 1] = TopoName{feature, Role::Side, src, (uint16_t)splitN[src]++};
    }
    return finishBody(solid, std::move(names), feature);
  }

  static TopoDS_Shape BRepBuilderAPI_Transform_Local(const TopoDS_Shape& s, const gp_Trsf& t) { return s.Moved(TopLoc_Location(t)); }

  KernelResult revolveImpl(const Profile2D& pr, const RevolveSpec& s, Uid feature) {
    if (pr.loops.empty() || pr.areaMm2 <= 0) return fail("The profile is empty.", "Select a closed region.");
    if (!finiteProfile(pr) || !std::isfinite(s.angleRad) || !finite(s.axisOrigin) || !finite(s.axisDir) || glm::length(s.axisDir) < 1e-12) return fail("The revolve axis is not valid.", "Pick an axis outside the profile.");
    if (std::abs(s.angleRad) < 1e-9) return fail("Angle is 0.", "Enter a non-zero angle.");
    Built b = buildFace(pr); if (!b.ok) return fail(b.err, "Check the sketch profile.");
    V3 ax = glm::normalize(s.axisDir);
    // reject an axis that crosses the profile
    double smin = 1e300, smax = -1e300; V3 side = glm::cross(ax, pr.plane.normal);
    for (auto& l : pr.loops) for (auto& e : l) for (V2 p : {e.a, e.b}) { V3 w = pr.plane.origin + pr.plane.xAxis * p.x + pr.plane.yAxis * p.y - s.axisOrigin; double sd = glm::dot(w, side); smin = std::min(smin, sd); smax = std::max(smax, sd); }
    if (smin < -1e-7 && smax > 1e-7) return fail("The axis crosses the profile.", "Pick an axis outside the profile.");
    gp_Ax1 axis(gp_Pnt(s.axisOrigin.x, s.axisOrigin.y, s.axisOrigin.z), D(ax));
    bool full = std::abs(s.angleRad) >= 2 * M_PI - 1e-9;
    BRepPrimAPI_MakeRevol rev = full ? BRepPrimAPI_MakeRevol(b.face, axis) : BRepPrimAPI_MakeRevol(b.face, axis, s.angleRad);
    if (!rev.IsDone()) return fail("The revolve failed.", "Change the angle or the axis.");
    TopoDS_Shape solid = rev.Shape(); NameMap names;
    TopTools_IndexedMapOfShape faces; TopExp::MapShapes(solid, TopAbs_FACE, faces);
    names.faces.assign(faces.Extent(), TopoName{feature, Role::Seam, {}, 0});
    if (!full) {
      if (int i = faces.FindIndex(rev.FirstShape())) names.faces[i - 1] = TopoName{feature, Role::RevCapStart, {}, 0};
      if (int i = faces.FindIndex(rev.LastShape())) names.faces[i - 1] = TopoName{feature, Role::RevCapEnd, {}, 0};
    }
    std::map<Uid, int> splitN;
    for (auto& [edge, src] : b.edges) {
      TopTools_ListOfShape gen = rev.Generated(edge);
      for (auto it = gen.begin(); it != gen.end(); ++it) if (int i = faces.FindIndex(*it)) names.faces[i - 1] = TopoName{feature, Role::RevSurface, src, (uint16_t)splitN[src]++};
    }
    return finishBody(solid, std::move(names), feature);
  }

  KernelResult booleanImpl(BoolOp op, BodyHandle target, BodyHandle tool, Uid feature) {
    Body *t = get(target), *u = get(tool);
    if (!t || !u) return fail("Reference lost: body.", "Pick the body again.");
    double v0 = props(t->shape).Mass();
    // Build-after-configure pattern: construct empty, set arguments / tools / operation / options, then Build().
    std::unique_ptr<BRepAlgoAPI_BooleanOperation> bop;
    if (op == BoolOp::Join) bop = std::make_unique<BRepAlgoAPI_Fuse>();
    else if (op == BoolOp::Cut) bop = std::make_unique<BRepAlgoAPI_Cut>();
    else bop = std::make_unique<BRepAlgoAPI_Common>();
    bop->SetOperation(op == BoolOp::Join ? BOPAlgo_FUSE : op == BoolOp::Cut ? BOPAlgo_CUT : BOPAlgo_COMMON);
    { TopTools_ListOfShape args, tools; args.Append(t->shape); tools.Append(u->shape); bop->SetArguments(args); bop->SetTools(tools); }
    // HYPOTHESIS U20 (spec 16 5.1.4): a fuzzy tolerance of 1e-7 mm absorbs the coincident-face noise that sketch-derived solids produce (flush
    // faces, shared edges) without merging real features; NOT yet measured on the Tab S7+ boolean corpus (KERN-4 golden + fuzz tests decide).
    // SetNonDestructive keeps both operands untouched, so a failed or rejected operation leaves the registered bodies exactly as they were.
    bop->SetFuzzyValue(1e-7);
    bop->SetNonDestructive(true);
    bop->Build();
    if (!bop->IsDone() || bop->HasErrors()) return fail("The boolean operation failed.", "Move the profile or change the distance.");
    TopoDS_Shape res = bop->Shape();
    double v1 = props(res).Mass();
    if (op == BoolOp::Cut && std::abs(v1 - v0) <= 1e-9 * std::max(1.0, v0)) return fail("Cut does not intersect the body.", "Move the profile or change the distance.");
    if (op == BoolOp::Intersect && v1 <= 1e-9) return fail("Intersect leaves nothing of the body.", "Move the profile onto the body, or choose New body.");   // disjoint operands: Common succeeds with an empty shape
    if (op == BoolOp::Join && std::abs(v1 - v0) <= 1e-9 * std::max(1.0, v0) && std::abs(v1 - (v0 + props(u->shape).Mass())) > 1e-6) { /* contained tool: still valid */ }
    NameMap names; TopTools_IndexedMapOfShape faces; TopExp::MapShapes(res, TopAbs_FACE, faces);
    names.faces.assign(faces.Extent(), TopoName{feature, Role::Side, {}, 0});
    std::vector<bool> named(faces.Extent(), false);
    auto carry = [&](Body* src) {
      for (int i = 1; i <= src->faces.Extent(); ++i) {
        const TopoDS_Shape& old = src->faces(i);
        if (bop->IsDeleted(old)) continue;
        TopTools_ListOfShape mods = bop->Modified(old);
        std::vector<int> targets;
        if (mods.IsEmpty()) { if (int k = faces.FindIndex(old)) targets.push_back(k); }
        else for (auto it = mods.begin(); it != mods.end(); ++it) if (int k = faces.FindIndex(*it)) targets.push_back(k);
        std::sort(targets.begin(), targets.end(), [&](int a, int b) { return centroidKey(faces(a)) < centroidKey(faces(b)); });
        uint16_t split = 0;
        for (int k : targets) if (!named[k - 1] && (size_t)i - 1 < src->names.faces.size()) { TopoName n = src->names.faces[i - 1]; n.split = (uint16_t)(n.split + split++); names.faces[k - 1] = n; named[k - 1] = true; }
      }
    };
    carry(t); carry(u);
    return finishBody(res, std::move(names), feature);
  }

  static std::array<long long, 3> centroidKey(const TopoDS_Shape& f) {
    GProp_GProps p; BRepGProp::SurfaceProperties(f, p); gp_Pnt c = p.CentreOfMass();
    return {llround(c.X() * 1e6), llround(c.Y() * 1e6), llround(c.Z() * 1e6)};
  }
  static GProp_GProps props(const TopoDS_Shape& s) { GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p; }

  KernelResult copyBodyImpl(BodyHandle h, Uid feature) {
    Body* b = get(h); if (!b) return fail("Reference lost: body.", "Pick the body again.");
    BRepBuilderAPI_Copy cp(b->shape, true);   // deep copy; face enumeration order is preserved, so names map by index
    TopoDS_Shape s = cp.Shape();
    NameMap names = b->names;
    for (auto& n : names.faces) n.feature = feature;
    for (auto& e : names.edges) { e.a.feature = feature; e.b.feature = feature; }
    TopTools_IndexedMapOfShape f2; TopExp::MapShapes(s, TopAbs_FACE, f2);
    if (f2.Extent() != b->faces.Extent()) return fail("The copy changed the face count.", "Try again.");
    return finishBody(s, std::move(names), feature);
  }
  std::optional<Aabb> boundsImpl(BodyHandle h) {
    Body* b = get(h); if (!b) return std::nullopt;
    Bnd_Box box; BRepBndLib::Add(b->shape, box); if (box.IsVoid()) return std::nullopt;
    double x0, y0, z0, x1, y1, z1; box.Get(x0, y0, z0, x1, y1, z1);
    Aabb a; a.min = V3(x0, y0, z0); a.max = V3(x1, y1, z1); return a;
  }

  MassProps massPropsImpl(BodyHandle h) {
    Body* b = get(h); if (!b) return {};
    GProp_GProps p = props(b->shape); gp_Pnt c = p.CentreOfMass(); return {p.Mass(), V3(c.X(), c.Y(), c.Z())};
  }
  bool validateImpl(BodyHandle h, std::string* why) {
    Body* b = get(h); if (!b) { if (why) *why = "unknown body"; return false; }
    BRepCheck_Analyzer an(b->shape);
    if (!an.IsValid() && why) *why = "BRepCheck reported an invalid shape";
    return an.IsValid();
  }

  std::shared_ptr<const MeshData> tessellateImpl(BodyHandle h, double chordTol, double angTol) {
    Body* b = get(h); if (!b) return nullptr;
    BRepMesh_IncrementalMesh mesher(b->shape, chordTol, false, angTol, true);
    auto m = std::make_shared<MeshData>();
    for (int fi = 1; fi <= b->faces.Extent(); ++fi) {
      TopoDS_Face face = TopoDS::Face(b->faces(fi)); TopLoc_Location loc;
      Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
      if (tri.IsNull()) continue;
      uint32_t base = (uint32_t)(m->pos.size() / 3);
      std::vector<V3> acc(tri->NbNodes(), V3(0));
      bool rev = face.Orientation() == TopAbs_REVERSED;
      for (int i = 1; i <= tri->NbNodes(); ++i) { gp_Pnt p = tri->Node(i).Transformed(loc); m->pos.insert(m->pos.end(), {(float)p.X(), (float)p.Y(), (float)p.Z()}); }
      for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int a, c2, d; tri->Triangle(i).Get(a, c2, d); if (rev) std::swap(c2, d);
        m->idx.insert(m->idx.end(), {base + a - 1, base + c2 - 1, base + d - 1}); m->triFace.push_back((uint32_t)(fi - 1));
        auto Pf = [&](int k) { return V3(m->pos[(base + k - 1) * 3], m->pos[(base + k - 1) * 3 + 1], m->pos[(base + k - 1) * 3 + 2]); };
        V3 n = glm::cross(Pf(c2) - Pf(a), Pf(d) - Pf(a));
        acc[a - 1] += n; acc[c2 - 1] += n; acc[d - 1] += n;
      }
      for (auto& n : acc) { double l = glm::length(n); if (l > 0) n /= l; m->nrm.insert(m->nrm.end(), {(float)n.x, (float)n.y, (float)n.z}); }
    }
    TopTools_IndexedMapOfShape edges; TopExp::MapShapes(b->shape, TopAbs_EDGE, edges);
    TopTools_IndexedDataMapOfShapeListOfShape edgeAnc; TopExp::MapShapesAndAncestors(b->shape, TopAbs_EDGE, TopAbs_FACE, edgeAnc);
    std::vector<uint32_t> edgeFaces;
    for (int ei = 1; ei <= edges.Extent(); ++ei) {
      TopLoc_Location loc; Handle(Poly_Polygon3D) poly = BRep_Tool::Polygon3D(TopoDS::Edge(edges(ei)), loc);
      if (poly.IsNull()) continue;
      for (int i = 1; i < poly->NbNodes(); ++i) { gp_Pnt a = poly->Nodes()(i).Transformed(loc), c = poly->Nodes()(i + 1).Transformed(loc);
        m->edgePos.insert(m->edgePos.end(), {(float)a.X(), (float)a.Y(), (float)a.Z(), (float)c.X(), (float)c.Y(), (float)c.Z()}); m->edgeId.push_back((uint32_t)ei);
        uint32_t fa = UINT32_MAX, fb = UINT32_MAX;
        if (int k = edgeAnc.FindIndex(edges(ei))) { for (auto fit = edgeAnc(k).begin(); fit != edgeAnc(k).end(); ++fit) { int fi = b->faces.FindIndex(*fit); if (!fi) continue; uint32_t v = (uint32_t)(fi - 1); if (fa == UINT32_MAX) fa = v; else if (v != fa && fb == UINT32_MAX) fb = v; } }
        edgeFaces.insert(edgeFaces.end(), {fa, fb}); }
    }
    buildFaceRanges(*m, (uint32_t)b->faces.Extent(), edgeFaces);
    m->version = h.id;
    return m;
  }

  std::optional<PlaneFrame> planarFaceImpl(BodyHandle h, const TopoName& n) {
    Body* b = get(h); if (!b) return std::nullopt; auto i = b->names.find(n); if (!i) return std::nullopt;
    TopoDS_Face f = TopoDS::Face(b->faces(*i + 1)); BRepAdaptor_Surface s(f);
    if (s.GetType() != GeomAbs_Plane) return std::nullopt;
    gp_Pln pl = s.Plane(); gp_Ax3 ax = pl.Position(); gp_Dir nd = ax.Direction(); if (f.Orientation() == TopAbs_REVERSED) nd.Reverse();
    PlaneFrame fr; fr.origin = V3(ax.Location().X(), ax.Location().Y(), ax.Location().Z()); fr.normal = V3(nd.X(), nd.Y(), nd.Z());
    fr.xAxis = V3(ax.XDirection().X(), ax.XDirection().Y(), ax.XDirection().Z()); fr.yAxis = glm::cross(fr.normal, fr.xAxis);
    return fr;
  }

  std::optional<Profile2D> faceProfileImpl(BodyHandle h, const TopoName& n) {
    auto fr = planarFaceImpl(h, n); if (!fr) return std::nullopt;
    Body* b = get(h); TopoDS_Face f = TopoDS::Face(b->faces(*b->names.find(n) + 1));
    Profile2D p; p.plane = *fr;
    auto to2 = [&](const gp_Pnt& q) { V3 w(q.X(), q.Y(), q.Z()); w -= fr->origin; return V2(glm::dot(w, fr->xAxis), glm::dot(w, fr->yAxis)); };
    for (TopExp_Explorer we(f, TopAbs_WIRE); we.More(); we.Next()) {
      Loop loop;
      for (TopExp_Explorer ee(we.Current(), TopAbs_EDGE); ee.More(); ee.Next()) {
        TopoDS_Edge e = TopoDS::Edge(ee.Current()); BRepAdaptor_Curve c(e);
        LoopEdge le; le.a = to2(c.Value(c.FirstParameter())); le.b = to2(c.Value(c.LastParameter()));
        if (e.Orientation() == TopAbs_REVERSED) std::swap(le.a, le.b);
        if (c.GetType() == GeomAbs_Circle) { gp_Circ ci = c.Circle(); le.kind = std::abs(c.LastParameter() - c.FirstParameter() - 2 * M_PI) < 1e-6 ? LoopEdge::Circle : LoopEdge::Arc; le.c = to2(ci.Location()); le.r = ci.Radius(); le.ccw = true; }
        loop.push_back(le);
      }
      p.loops.push_back(loop);
    }
    p.areaMm2 = 0; if (!p.loops.empty()) { p.areaMm2 = std::abs(loopSignedArea(p.loops[0])); for (size_t i = 1; i < p.loops.size(); ++i) p.areaMm2 -= std::abs(loopSignedArea(p.loops[i])); }
    GProp_GProps sp; BRepGProp::SurfaceProperties(f, sp); gp_Pnt cm = sp.CentreOfMass(); p.centroid = to2(cm);
    return p;
  }

  std::vector<V3> snapPointsImpl(BodyHandle h) {
    Body* b = get(h); std::vector<V3> out; if (!b) return out;
    TopTools_IndexedMapOfShape vs, es; TopExp::MapShapes(b->shape, TopAbs_VERTEX, vs); TopExp::MapShapes(b->shape, TopAbs_EDGE, es);
    for (int i = 1; i <= vs.Extent(); ++i) { gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vs(i))); out.emplace_back(p.X(), p.Y(), p.Z()); }
    for (int i = 1; i <= es.Extent(); ++i) { BRepAdaptor_Curve c(TopoDS::Edge(es(i))); gp_Pnt p = c.Value((c.FirstParameter() + c.LastParameter()) / 2); out.emplace_back(p.X(), p.Y(), p.Z()); }
    return out;
  }

  // ---- Public interface (KERN-2, spec 16 5.1.4 / 6.1): every entry point is guarded so an OCCT exception (Standard_Failure, or anything else)
  // can never escape into the app. Reasons are engine-free (no "OCCT" in user-facing text, spec 17 S10); the raw exception text is dropped on purpose.
  KernelResult extrude(const Profile2D& pr, const ExtrudeSpec& s, Uid feature) override {
    try { return extrudeImpl(pr, s, feature); }
    catch (const Standard_Failure&) { return fail("The extrusion failed.", "Change the distance or the profile."); }
    catch (...) { return fail("The extrusion failed.", "Change the distance or the profile."); }
  }
  KernelResult revolve(const Profile2D& pr, const RevolveSpec& s, Uid feature) override {
    try { return revolveImpl(pr, s, feature); }
    catch (const Standard_Failure&) { return fail("The revolve failed.", "Change the angle or the axis."); }
    catch (...) { return fail("The revolve failed.", "Change the angle or the axis."); }
  }
  KernelResult boolean(BoolOp op, BodyHandle target, BodyHandle tool, Uid feature) override {
    try { return booleanImpl(op, target, tool, feature); }
    catch (const Standard_Failure&) { return fail("The boolean operation failed.", "Move the profile or change the distance."); }
    catch (...) { return fail("The boolean operation failed.", "Move the profile or change the distance."); }
  }
  KernelResult copyBody(BodyHandle h, Uid feature) override {
    try { return copyBodyImpl(h, feature); }
    catch (const Standard_Failure&) { return fail("The copy failed.", "Try again."); }
    catch (...) { return fail("The copy failed.", "Try again."); }
  }
  std::optional<Aabb> bounds(BodyHandle h) override {
    try { return boundsImpl(h); } catch (const Standard_Failure&) { return std::nullopt; } catch (...) { return std::nullopt; }
  }
  MassProps massProps(BodyHandle h) override {
    try { return massPropsImpl(h); } catch (const Standard_Failure&) { return {}; } catch (...) { return {}; }
  }
  bool validate(BodyHandle h, std::string* why) override {
    try { return validateImpl(h, why); }
    catch (const Standard_Failure&) { if (why) *why = "The body could not be checked."; return false; }
    catch (...) { if (why) *why = "The body could not be checked."; return false; }
  }
  std::shared_ptr<const MeshData> tessellate(BodyHandle h, double chordTol, double angTol) override {
    try { return tessellateImpl(h, chordTol, angTol); } catch (const Standard_Failure&) { return nullptr; } catch (...) { return nullptr; }
  }
  std::optional<PlaneFrame> planarFace(BodyHandle h, const TopoName& n) override {
    try { return planarFaceImpl(h, n); } catch (const Standard_Failure&) { return std::nullopt; } catch (...) { return std::nullopt; }
  }
  std::optional<Profile2D> faceProfile(BodyHandle h, const TopoName& n) override {
    try { return faceProfileImpl(h, n); } catch (const Standard_Failure&) { return std::nullopt; } catch (...) { return std::nullopt; }
  }
  std::vector<V3> snapPoints(BodyHandle h) override {
    try { return snapPointsImpl(h); } catch (const Standard_Failure&) { return {}; } catch (...) { return {}; }
  }
  void release(BodyHandle h) override {
    try { std::lock_guard<std::mutex> lk(m_); bodies_.erase(h.id); } catch (const Standard_Failure&) {} catch (...) {}
  }

 private:
  Body* get(BodyHandle h) { std::lock_guard<std::mutex> lk(m_); auto it = bodies_.find(h.id); return it == bodies_.end() ? nullptr : &it->second; }
  std::mutex m_; std::unordered_map<uint64_t, Body> bodies_; uint64_t next_ = 0;
};
}  // namespace

std::unique_ptr<IGeometryKernel> makeOcctKernel() { return std::make_unique<OcctKernel>(); }

}  // namespace chisel
