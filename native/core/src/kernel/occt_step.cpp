// SPDX-License-Identifier: LGPL-2.1-only
// KERN-6 (U38, G4 spec 8.1c): STEP writer of the OCCT adapter. Part of the published LGPL source set.
// Plain STEPControl_Writer (TKDESTEP + TKXSBase + TKDE, no XCAF): one root per body, product name per body, no colours.
// No OCCT type crosses the public header: occt_kernel.cpp hands over (shape, name) pairs and gets a StepResult back.
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <string>
#include <system_error>
#include <utility>
#include <vector>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Standard_Failure.hxx>
#include <STEPControl_Controller.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <StepBasic_Product.hxx>
#include <StepData_StepModel.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TopoDS_Shape.hxx>
#include "chisel/kernel/geometry_kernel.h"

namespace chisel::stepexport {
namespace {
constexpr const char* kFailError = "The STEP file could not be made.";   // Product S89, verbatim (docs/requirements/r2/prd_cad8.md)
constexpr const char* kFailFix = "Try again, or export 3MF.";
constexpr size_t kMaxNameBytes = 128;

StepResult failed(const char* error = kFailError, const char* fix = kFailFix) { StepResult r; r.error = error; r.fix = fix; return r; }

// Control characters removed, ends trimmed, capped at 128 bytes on a UTF-8 boundary, empty -> "Body N" (1-based).
std::string cleanName(const std::string& raw, size_t index) {
  std::string s; s.reserve(raw.size());
  for (unsigned char c : raw) if (c >= 0x20 && c != 0x7f) s.push_back(static_cast<char>(c));
  while (!s.empty() && s.back() == ' ') s.pop_back();
  size_t b = 0; while (b < s.size() && s[b] == ' ') ++b;
  s.erase(0, b);
  if (s.size() > kMaxNameBytes) { size_t n = kMaxNameBytes; while (n > 0 && (static_cast<unsigned char>(s[n]) & 0xC0) == 0x80) --n; s.resize(n); }
  if (s.empty()) s = "Body " + std::to_string(index + 1);
  return s;
}

// Interface_Static is process-wide state: one writer at a time.
std::mutex gStepMutex;
std::once_flag gInitOnce;
}  // namespace

// C8-G8e (spec 17 G8 4.3 step 1): the STEP reader (occt_step_import.cpp) takes the SAME mutex and the same one-time Init (xstep.* statics are process-wide).
std::mutex& stepMutex() { return gStepMutex; }
void stepInitOnce() { std::call_once(gInitOnce, [] { STEPControl_Controller::Init(); }); }

// `shapes`: one (shape, name) per body, in order. The file is written to `<path>.part` and renamed on success, so a failure never leaves a partial file
// and never destroys an older file at `path`.
StepResult write(const std::vector<std::pair<TopoDS_Shape, std::string>>& shapes, const StepSpec& spec) {
  if (shapes.empty() || spec.path.empty()) return failed();
  const std::string part = spec.path + ".part";
  std::error_code ec;
  try {
    std::lock_guard<std::mutex> lk(gStepMutex);
    // Init() FIRST: the first STEPControl_Writer construction runs it and would reset write.step.schema to the default (spike C8-R-FMT).
    std::call_once(gInitOnce, [] { STEPControl_Controller::Init(); });
    if (!Interface_Static::SetCVal("write.step.schema", spec.schema.c_str())) return failed();
    if (!Interface_Static::SetCVal("write.step.unit", "MM")) return failed();
    if (!Interface_Static::SetIVal("write.surfacecurve.mode", 1)) return failed();
    STEPControl_Writer w;
    for (size_t i = 0; i < shapes.size(); ++i) {
      std::string nm = cleanName(shapes[i].second, i);
      if (!Interface_Static::SetCVal("write.step.product.name", nm.c_str())) return failed();
      if (shapes[i].first.IsNull()) return failed();
      const int before = w.Model()->NbEntities();
      if (w.Transfer(shapes[i].first, STEPControl_AsIs) != IFSelect_RetDone) return failed();
      // OCCT appends its assembly level to write.step.product.name ("BodyA 1"): set the exact body name on the PRODUCT entities this transfer created.
      occ::handle<StepData_StepModel> model = w.Model();
      for (int e = before + 1; e <= model->NbEntities(); ++e)
        if (occ::handle<StepBasic_Product> p = occ::down_cast<StepBasic_Product>(model->Value(e))) {
          p->SetId(new TCollection_HAsciiString(nm.c_str())); p->SetName(new TCollection_HAsciiString(nm.c_str()));
        }
    }
    if (w.Write(part.c_str()) != IFSelect_RetDone) { std::filesystem::remove(part, ec); return failed(); }
  } catch (const Standard_Failure&) { std::filesystem::remove(part, ec); return failed(); }
  catch (...) { std::filesystem::remove(part, ec); return failed(); }
  std::filesystem::rename(part, spec.path, ec);
  if (ec) { std::filesystem::remove(part, ec); return failed(); }
  StepResult r; r.ok = true; r.bodies = static_cast<int>(shapes.size());
  auto sz = std::filesystem::file_size(spec.path, ec); r.bytes = ec ? 0 : static_cast<uint64_t>(sz);
  return r;
}
}  // namespace chisel::stepexport
