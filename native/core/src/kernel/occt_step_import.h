// SPDX-License-Identifier: LGPL-2.1-only
#pragma once
// C8-G8e (spec 17 G8 4.3): internal interface between occt_step_import.cpp (the STEP read + entity walk) and occt_kernel.cpp (body registration,
// face names, limits per solid, cancel between solids). Part of the published LGPL source set. Never included by the core outside the adapter.
#include <cstdint>
#include <string>
#include <vector>
#include <TopoDS_Shape.hxx>
#include "chisel/kernel/geometry_kernel.h"

namespace chisel::stepimport {

struct Solid {
  TopoDS_Shape shape;                       // a valid TopAbs_SOLID (healed once if needed), in the deterministic order of 4.3 step 6
  std::string name; bool hasColour = false; float rgb[3] = {0, 0, 0};
  std::vector<std::string> layers; int node = -1;
};
struct Read {
  bool ok = false; StepImportResult::Status status = StepImportResult::Unreadable; std::string error;
  std::vector<Solid> solids; std::vector<StepImportNode> nodes;
  int nonSolids = 0; uint32_t dropped = 0;
};

// Parses data/size (units forced to millimetres on every call, under the shared STEP mutex), transfers the roots (cancel polled through
// spec.cancelled), splits them into solids, walks the entities for names / colours / layers / assembly nodes. Never throws.
// Enforces maxSolids; maxFacesPerSolid and the between-solid cancel are the caller's (they happen while bodies are registered).
Read read(const StepImportSpec& spec);

}  // namespace chisel::stepimport
