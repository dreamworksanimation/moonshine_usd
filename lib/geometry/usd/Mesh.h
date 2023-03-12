// Copyright 2023 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#pragma once

#include "Common.h"

#include <moonray/rendering/geom/Api.h>
#include <moonray/rendering/geom/ProceduralContext.h>
#include <scene_rdl2/scene/rdl2/rdl2.h>
#include <scene_rdl2/common/platform/Platform.h>

#include <pxr/pxr.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usdGeom/mesh.h>

#include <memory>
#include <vector>

using namespace moonray;
using namespace moonray::geom;

PXR_NAMESPACE_USING_DIRECTIVE

namespace moonshine {
namespace usd {

enum class SubdType {
    USE_MESH_TYPE,
    FORCE_POLYGONMESH,
    FORCE_SUBDIVISIONMESH
};

struct MeshAttrs {
    float meshResolution;
    float adaptiveError;
    SubdType subdType;
    bool smoothNormal;

    MeshAttrs()
        : meshResolution(1.0f)
        , adaptiveError(0.0f)
        , subdType(SubdType::USE_MESH_TYPE)
        , smoothNormal(false)
    {}
};

/*
 * Create either a PolygonMesh or a SubdivisionMesh from a
 * single Mesh prim
 */
std::unique_ptr <Primitive>
createMesh(const scene_rdl2::rdl2::Geometry *rdlGeometry,
           const GenerateContext &generateContext,
           std::vector<float>& motionFrames,
           const UsdGeomMesh &mesh,
           const scene_rdl2::rdl2::Layer *rdlLayer,
           const CommonAttrs& commonAttrs,
           const MeshAttrs& meshAttrs);

} // namespace usd
} // namespace moonshine

