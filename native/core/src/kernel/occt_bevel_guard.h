// SPDX-License-Identifier: LGPL-2.1-only
// KERN-5 (ADR-027, G2 spec 4.1b/4.1c): PRIVATE helpers shared by chamferEdges (occt_chamfer.cpp), draftFaces (occt_draft.cpp) and the OCCT adapter.
// Part of the published LGPL source set. No OCCT type crosses the public header; this header is included only by the adapter's translation units.
#pragma once
#include <string>
#include <vector>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include "chisel/kernel/geometry_kernel.h"

namespace chisel::bevel {

constexpr double kMinChamferMm = 1e-3;   // HYPOTHESIS (G2 4.1b): 1e-6 is a silent no-op (measured); 1e-3 mm is the finest custom export chord
constexpr double kMinDraftRad = 1e-4;
constexpr double kMaxDraftRad = 1.5;

// Result of a bevel operation before the adapter registers the body: the new shape plus one TopoName per face (MapShapes face order).
struct Out { bool ok = false; std::string error, fix; TopoDS_Shape shape; std::vector<TopoName> faceNames; };
Out refuse(const char* error, const char* fix);

// The body an operation works on: a COPY of the input (optionally unified, G2 4.1c) with maps in MapShapes order and parallel names.
struct Work {
  TopoDS_Shape shape; TopTools_IndexedMapOfShape faces, edges;
  std::vector<TopoName> faceNames; std::vector<EdgeName> edgeNames;
  std::vector<int> edgeOfInput, faceOfInput;   // input edge/face index (0-based) -> working index, -1 when the entity vanished (a seam)
};

bool finiteV(const glm::dvec3& v);
double volumeGK(const TopoDS_Shape& s);        // accurate Gauss-Kronrod volume (BRepGProp::VolumePropertiesGK, eps 1e-9)
bool isValid(const TopoDS_Shape& s);           // BRepCheck_Analyzer
int faceCount(const TopoDS_Shape& s);
bool sameEdgeName(const EdgeName& a, const EdgeName& b);

// Names of every edge of `shape`: (sorted adjacent face names, tie = rank by midpoint among edges with the same pair). Parallel to `edges`.
std::vector<EdgeName> edgeNamesOf(const TopTools_IndexedMapOfShape& faces, const TopTools_IndexedMapOfShape& edges, const std::vector<TopoName>& faceNames);

enum class Convexity { Convex, Concave, Smooth, Other };   // Other: seam / free / non-manifold / not computable
// `anc` = TopExp::MapShapesAndAncestors(shape, EDGE, FACE). Smooth = both faces have the same normal at the edge midpoint (a coplanar seam left by a plain Join is Smooth too).
Convexity edgeConvexity(const TopTools_IndexedDataMapOfShapeListOfShape& anc, const TopoDS_Edge& e);

// Copy `shape`, run ShapeUpgrade_UnifySameDomain on the copy with history, map names (merged face = smallest predecessor name, G2 4.1c).
// `inEdges`/`inFaces` are the maps of the INPUT body (same order as its names). Returns false only on an internal failure (no result).
bool makeWork(const TopoDS_Shape& shape, const TopTools_IndexedMapOfShape& inFaces, const TopTools_IndexedMapOfShape& inEdges,
              const std::vector<TopoName>& inFaceNames, Work& out);

// The registered input body as the adapter holds it (read only).
struct Input { TopoDS_Shape shape; TopTools_IndexedMapOfShape faces, edges; std::vector<TopoName> faceNames; std::vector<EdgeName> edgeNames; };
Out chamfer(const Input& in, const ChamferSpec& spec, Uid feature);   // occt_chamfer.cpp (never throws)
Out draft(const Input& in, const DraftSpec& spec, Uid feature);       // occt_draft.cpp (never throws)
Out bend(const Input& in, const BendSpec& spec, Uid feature, BendReport* rep);   // occt_bend.cpp (F3, never throws; rep may be null)

// Deterministic, platform independent (FNV-1a) id of an edge name; the name of a face made by a chamfer: {feature, Side, edgeUid, tie}.
Uid edgeUid(const EdgeName& e);
void makeNamesUnique(std::vector<TopoName>& names);   // a duplicate gets split+1, +2 ... so two faces never share a name

}  // namespace chisel::bevel
