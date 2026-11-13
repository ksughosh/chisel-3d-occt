// SPDX-License-Identifier: LGPL-2.1-only
// C8-G8e (spec 17 G8 4.3): STEP exact import of the OCCT adapter (route 1b: plain STEPControl_Reader + a read-only walk of the STEP entities;
// no XCAF toolkit). Part of the published LGPL source set. occt_kernel.cpp::importStep registers the solids this file returns as bodies.
// No OCCT type crosses the public header.
#include <array>
#include <climits>
#include <cstring>
#include <istream>
#include <map>
#include <mutex>
#include <streambuf>
#include <string>
#include <vector>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>
#include <Message_ProgressScope.hxx>
#include <ShapeFix_Shape.hxx>
#include <Standard_Failure.hxx>
#include <STEPControl_Reader.hxx>
#include <StepBasic_Product.hxx>
#include <StepBasic_ProductDefinition.hxx>
#include <StepBasic_ProductDefinitionFormation.hxx>
#include <StepData_StepModel.hxx>
#include <StepRepr_NextAssemblyUsageOccurrence.hxx>
#include <StepRepr_ProductDefinitionShape.hxx>
#include <StepRepr_PropertyDefinition.hxx>
#include <StepRepr_Representation.hxx>
#include <StepRepr_RepresentationItem.hxx>
#include <StepRepr_RepresentationRelationshipWithTransformation.hxx>
#include <StepRepr_ShapeRepresentationRelationship.hxx>
#include <StepShape_ManifoldSolidBrep.hxx>
#include <StepShape_ShapeDefinitionRepresentation.hxx>
#include <StepVisual_ColourRgb.hxx>
#include <StepVisual_DraughtingPreDefinedColour.hxx>
#include <StepVisual_FillAreaStyle.hxx>
#include <StepVisual_FillAreaStyleColour.hxx>
#include <StepVisual_LayeredItem.hxx>
#include <StepVisual_PreDefinedItem.hxx>
#include <StepVisual_PresentationLayerAssignment.hxx>
#include <StepVisual_PresentationStyleAssignment.hxx>
#include <StepVisual_StyledItem.hxx>
#include <StepVisual_SurfaceSideStyle.hxx>
#include <StepVisual_SurfaceStyleFillArea.hxx>
#include <StepVisual_SurfaceStyleUsage.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <TransferBRep.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>
#include "occt_step_import.h"

namespace chisel::stepexport { std::mutex& stepMutex(); void stepInitOnce(); }   // occt_step.cpp: one STEP mutex for reader and writer (xstep.* statics)

namespace chisel::stepimport {
namespace {
// A read-only std::streambuf over the caller's bytes (no copy; spec 4.3 step 3). size <= INT_MAX is checked before (A-5).
struct MemBuf : std::streambuf {
  MemBuf(const uint8_t* d, size_t n) { char* p = const_cast<char*>(reinterpret_cast<const char*>(d)); setg(p, p, p + n); }
  pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override {
    if (!(which & std::ios_base::in)) return pos_type(off_type(-1));
    char* base = dir == std::ios_base::beg ? eback() : dir == std::ios_base::cur ? gptr() : egptr();
    char* p = base + off; if (p < eback() || p > egptr()) return pos_type(off_type(-1));
    setg(eback(), p, egptr()); return pos_type(p - eback());
  }
  pos_type seekpos(pos_type pos, std::ios_base::openmode which) override { return seekoff(off_type(pos), std::ios_base::beg, which); }
};
// TransferRoots polls UserBreak(): the caller's C callback (spec 4.2: no <atomic> in the header).
struct Breaker : Message_ProgressIndicator {
  const StepImportSpec* s;
  explicit Breaker(const StepImportSpec* sp) : s(sp) {}
  bool UserBreak() override { return s->cancelled && s->cancelled(s->user); }
  void Show(const Message_ProgressScope&, const bool) override {}
};
bool cancelled(const StepImportSpec& s) { return s.cancelled && s.cancelled(s.user); }
Read fail(StepImportResult::Status st, const char* why) { Read r; r.status = st; r.error = why; return r; }
std::string str(const occ::handle<TCollection_HAsciiString>& h) { return h.IsNull() ? std::string() : std::string(h->ToCString()); }

// Solid-level colour of a STYLED_ITEM (COLOUR_RGB, or a DRAUGHTING_PRE_DEFINED_COLOUR name: OCCT writes the pure colours that way, spike finding).
bool styleColour(const occ::handle<StepVisual_StyledItem>& sty, float rgb[3]) {
  static const std::map<std::string, std::array<float, 3>> kNamed = {{"red", {1, 0, 0}}, {"green", {0, 1, 0}}, {"blue", {0, 0, 1}}, {"yellow", {1, 1, 0}},
                                                                     {"cyan", {0, 1, 1}}, {"magenta", {1, 0, 1}}, {"black", {0, 0, 0}}, {"white", {1, 1, 1}}};
  for (int i = 1; i <= sty->NbStyles(); ++i) {
    occ::handle<StepVisual_PresentationStyleAssignment> psa = sty->StylesValue(i); if (psa.IsNull()) continue;
    for (int j = 1; j <= psa->NbStyles(); ++j) {
      occ::handle<StepVisual_SurfaceStyleUsage> ssu = psa->StylesValue(j).SurfaceStyleUsage(); if (ssu.IsNull() || ssu->Style().IsNull()) continue;
      occ::handle<StepVisual_SurfaceSideStyle> side = ssu->Style();
      for (int k = 1; k <= side->NbStyles(); ++k) {
        occ::handle<StepVisual_SurfaceStyleFillArea> fa = side->StylesValue(k).SurfaceStyleFillArea(); if (fa.IsNull() || fa->FillArea().IsNull()) continue;
        occ::handle<StepVisual_FillAreaStyle> fas = fa->FillArea();
        for (int q = 1; q <= fas->NbFillStyles(); ++q) {
          occ::handle<StepVisual_FillAreaStyleColour> fc = fas->FillStylesValue(q).FillAreaStyleColour(); if (fc.IsNull()) continue;
          occ::handle<StepVisual_Colour> col = fc->FillColour();
          if (occ::handle<StepVisual_ColourRgb> c = occ::down_cast<StepVisual_ColourRgb>(col)) { rgb[0] = (float)c->Red(); rgb[1] = (float)c->Green(); rgb[2] = (float)c->Blue(); return true; }
          if (occ::handle<StepVisual_DraughtingPreDefinedColour> pc = occ::down_cast<StepVisual_DraughtingPreDefinedColour>(col)) {
            if (pc->GetPreDefinedItem().IsNull()) continue;
            auto it = kNamed.find(str(pc->GetPreDefinedItem()->Name())); if (it == kNamed.end()) continue;
            rgb[0] = it->second[0]; rgb[1] = it->second[1]; rgb[2] = it->second[2]; return true;
          }
        }
      }
    }
  }
  return false;
}

// Top-level split (4.3 step 6): compounds and compsolids are descended; every solid is kept in TopExp_Explorer order; every other top-level item
// (shell, face, wire, edge, vertex) counts ONE non-solid (the spike's "solid + loose face = 1"). Sheets never become bodies (M15).
void split(const TopoDS_Shape& s, std::vector<TopoDS_Shape>& solids, int& nonSolids) {
  if (s.IsNull()) return;
  const TopAbs_ShapeEnum t = s.ShapeType();
  if (t == TopAbs_COMPOUND || t == TopAbs_COMPSOLID) { for (TopoDS_Iterator it(s); it.More(); it.Next()) split(it.Value(), solids, nonSolids); return; }
  if (t == TopAbs_SOLID) solids.push_back(s); else ++nonSolids;
}
// Valid as read, else ShapeFix_Shape once; a solid still invalid (or that heals into anything but one solid) is counted as a non-solid (4.3 step 9).
bool healed(const TopoDS_Shape& in, TopoDS_Shape& out) {
  if (BRepCheck_Analyzer(in).IsValid()) { out = in; return true; }
  ShapeFix_Shape fix(in); fix.Perform(); TopoDS_Shape f = fix.Shape();
  std::vector<TopoDS_Shape> s; int other = 0; split(f, s, other);
  if (s.size() != 1 || other != 0 || !BRepCheck_Analyzer(s[0]).IsValid()) return false;
  out = s[0]; return true;
}
// Product of a product definition (Name, else Id).
std::string productName(const occ::handle<StepBasic_ProductDefinition>& pd) {
  if (pd.IsNull() || pd->Formation().IsNull() || pd->Formation()->OfProduct().IsNull()) return {};
  occ::handle<StepBasic_Product> p = pd->Formation()->OfProduct(); std::string n = str(p->Name()); return n.empty() ? str(p->Id()) : n;
}
}  // namespace

namespace {
// Labels found by the entity walk (4.3 step 4), keyed by the shape each STEP solid entity transferred to; matched to the final solids by IsPartner (G8-D9:
// an instanced part is the SAME TShape under another location, so IsSame would miss every placed copy: M17).
struct Label { TopoDS_Shape shape; int node = -1; bool hasColour = false; float rgb[3] = {0, 0, 0}; std::vector<std::string> layers; };

struct Walk {
  occ::handle<StepData_StepModel> m; occ::handle<Transfer_TransientProcess> tp;
  std::vector<StepImportNode> nodes; std::map<int, int> nodeOfPd;                  // product definition entity number -> node index
  std::map<int, int> pdOfRep;                                                     // representation entity number -> product definition entity number
  std::vector<Label> labels; std::map<int, size_t> labelOfItem;                   // solid entity number -> label
  uint32_t dropped = 0;

  int nodeFor(const occ::handle<StepBasic_ProductDefinition>& pd) {
    if (pd.IsNull()) return -1; const int e = m->Number(pd); auto it = nodeOfPd.find(e); if (it != nodeOfPd.end()) return it->second;
    StepImportNode n; n.name = productName(pd); nodes.push_back(n); return nodeOfPd[e] = (int)nodes.size() - 1;
  }
  TopoDS_Shape shapeOf(const occ::handle<Standard_Transient>& e) const { return e.IsNull() ? TopoDS_Shape() : TransferBRep::ShapeResult(tp, e); }
  Label* labelFor(const occ::handle<Standard_Transient>& item) {
    if (item.IsNull()) return nullptr; auto it = labelOfItem.find(m->Number(item)); return it == labelOfItem.end() ? nullptr : &labels[it->second];
  }

  void run() {
    const int n = m->NbEntities();
    // 1. products and the assembly tree (a product used twice is ONE node; its first parent wins)
    for (int e = 1; e <= n; ++e) if (auto pd = occ::down_cast<StepBasic_ProductDefinition>(m->Value(e))) nodeFor(pd);
    for (int e = 1; e <= n; ++e) if (auto u = occ::down_cast<StepRepr_NextAssemblyUsageOccurrence>(m->Value(e))) {
      const int parent = nodeFor(u->RelatingProductDefinition()), child = nodeFor(u->RelatedProductDefinition());
      if (parent >= 0 && child >= 0 && child != parent && nodes[child].parent < 0) {
        bool cycle = false; for (int p = parent; p >= 0; p = nodes[p].parent) if (p == child) { cycle = true; break; }
        if (!cycle) nodes[child].parent = parent;
      }
    }
    // 2. representation -> product definition (SHAPE_DEFINITION_REPRESENTATION), spread through plain SHAPE_REPRESENTATION_RELATIONSHIPs (not the placed ones)
    for (int e = 1; e <= n; ++e) if (auto sdr = occ::down_cast<StepShape_ShapeDefinitionRepresentation>(m->Value(e))) {
      occ::handle<StepRepr_PropertyDefinition> pdef = sdr->Definition().PropertyDefinition(); if (pdef.IsNull() || sdr->UsedRepresentation().IsNull()) continue;
      occ::handle<StepBasic_ProductDefinition> pd = pdef->Definition().ProductDefinition(); if (pd.IsNull()) continue;
      pdOfRep[m->Number(sdr->UsedRepresentation())] = m->Number(pd);
    }
    for (int pass = 0; pass < 4; ++pass)
      for (int e = 1; e <= n; ++e) if (auto rr = occ::down_cast<StepRepr_ShapeRepresentationRelationship>(m->Value(e))) {
        if (occ::down_cast<StepRepr_RepresentationRelationshipWithTransformation>(rr) || rr->Rep1().IsNull() || rr->Rep2().IsNull()) continue;
        const int a = m->Number(rr->Rep1()), b = m->Number(rr->Rep2());
        if (pdOfRep.count(a) && !pdOfRep.count(b)) pdOfRep[b] = pdOfRep[a]; else if (pdOfRep.count(b) && !pdOfRep.count(a)) pdOfRep[a] = pdOfRep[b];
      }
    // 3. every solid entity inside a representation: its transferred shape + its product's node
    for (int e = 1; e <= n; ++e) if (auto rep = occ::down_cast<StepRepr_Representation>(m->Value(e))) {
      auto pdIt = pdOfRep.find(e); const int node = pdIt == pdOfRep.end() ? -1 : nodeFor(occ::down_cast<StepBasic_ProductDefinition>(m->Value(pdIt->second)));
      for (int i = 1; i <= rep->NbItems(); ++i) {
        occ::handle<StepRepr_RepresentationItem> it = rep->ItemsValue(i); if (it.IsNull() || !occ::down_cast<StepShape_ManifoldSolidBrep>(it)) continue;
        if (Label* have = labelFor(it)) { if (have->node < 0) have->node = node; continue; }
        TopoDS_Shape s = shapeOf(it); if (s.IsNull()) continue;
        Label l; l.shape = s; l.node = node; labelOfItem[m->Number(it)] = labels.size(); labels.push_back(std::move(l));
      }
    }
    // 4. solid-level colours (STYLED_ITEM on a solid entity); a colour on anything else (faces, shells) is dropped (bit 0)
    for (int e = 1; e <= n; ++e) if (auto sty = occ::down_cast<StepVisual_StyledItem>(m->Value(e))) {
      float rgb[3]; if (!styleColour(sty, rgb)) continue;
      Label* l = labelFor(sty->Item());
      if (!l) { dropped |= 1u; continue; }
      if (!l->hasColour) { l->hasColour = true; l->rgb[0] = rgb[0]; l->rgb[1] = rgb[1]; l->rgb[2] = rgb[2]; }
    }
    // 5. presentation layers on solid entities, file order; assignments to anything else are dropped (bit 1)
    for (int e = 1; e <= n; ++e) if (auto pl = occ::down_cast<StepVisual_PresentationLayerAssignment>(m->Value(e))) {
      const std::string nm = str(pl->Name());
      for (int i = 1; i <= pl->NbAssignedItems(); ++i) {
        Label* l = labelFor(pl->AssignedItemsValue(i).Value());
        if (!l || nm.empty()) { dropped |= 2u; continue; }
        bool dup = false; for (auto& x : l->layers) dup = dup || x == nm; if (!dup) l->layers.push_back(nm);
      }
    }
  }
};
}  // namespace

Read read(const StepImportSpec& spec) {
  if (spec.data == nullptr || spec.size == 0 || spec.size > (size_t)INT_MAX) return fail(StepImportResult::Unreadable, "empty or oversized input");   // A-5: before any OCCT call
  try {
    std::lock_guard<std::mutex> lk(stepexport::stepMutex());
    stepexport::stepInitOnce();
    // 4.3 step 2, EVERY call: another caller may have left "M" (volumes 1e9 off) or "INCH" (M14).
    if (!Interface_Static::SetCVal("xstep.cascade.unit", "MM")) return fail(StepImportResult::Unreadable, "unit");
    MemBuf buf(spec.data, spec.size); std::istream is(&buf);
    STEPControl_Reader r;
    if (r.ReadStream("file.step", is) != IFSelect_RetDone) return fail(StepImportResult::Unreadable, "ReadStream failed");
    if (cancelled(spec)) return fail(StepImportResult::Cancelled, "cancelled");
    if (r.NbRootsForTransfer() <= 0) return fail(StepImportResult::Unreadable, "no roots");
    occ::handle<Breaker> brk = new Breaker(&spec);
    { Message_ProgressRange range = brk->Start(); r.TransferRoots(range); }
    if (cancelled(spec)) return fail(StepImportResult::Cancelled, "cancelled");
    Read out;
    std::vector<TopoDS_Shape> raw;
    for (int i = 1; i <= r.NbShapes(); ++i) split(r.Shape(i), raw, out.nonSolids);
    if (raw.size() > spec.maxSolids) return fail(StepImportResult::TooManySolids, "too many solids");   // before building anything (4.3 step 8)
    Walk w; w.m = r.StepModel(); w.tp = r.WS()->TransferReader()->TransientProcess();
    if (!w.m.IsNull() && !w.tp.IsNull()) w.run();
    out.dropped = w.dropped;
    for (auto& s : raw) {
      if (cancelled(spec)) return fail(StepImportResult::Cancelled, "cancelled");
      TopoDS_Shape good; if (!healed(s, good)) { ++out.nonSolids; continue; }
      Solid so; so.shape = good;
      for (auto& l : w.labels) if (l.shape.IsPartner(s)) {   // IsPartner: same TShape, any location (an instanced part keeps its labels)
        so.hasColour = l.hasColour; for (int k = 0; k < 3; ++k) so.rgb[k] = l.rgb[k]; so.layers = l.layers; so.node = l.node;
        if (so.node >= 0) so.name = w.nodes[so.node].name;
        break;
      }
      if (so.node < 0) for (size_t k = 0; k < w.nodes.size(); ++k) if (w.nodes[k].parent < 0) { so.node = (int)k; break; }   // unmatched -> the root product, no name
      out.solids.push_back(std::move(so));
    }
    out.nodes = std::move(w.nodes);
    if (out.solids.empty()) { Read z = fail(StepImportResult::NoSolids, "no solids"); z.nonSolids = out.nonSolids; return z; }
    out.ok = true; out.status = StepImportResult::Ok; return out;
  } catch (const Standard_Failure& f) { return fail(StepImportResult::Unreadable, f.what() ? f.what() : "Standard_Failure"); }
  catch (...) { return fail(StepImportResult::Unreadable, "exception"); }
}

}  // namespace chisel::stepimport
