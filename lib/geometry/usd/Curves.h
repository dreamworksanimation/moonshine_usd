// Copyright 2023-2024 DreamWorks Animation LLC
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
#include <pxr/usd/usdGeom/basisCurves.h>

#include <memory>
#include <vector>

using namespace moonray;
using namespace moonray::geom;

PXR_NAMESPACE_USING_DIRECTIVE

namespace moonshine {
namespace usd {

std::unique_ptr <Primitive>
createCurves(const scene_rdl2::rdl2::Geometry *rdlGeometry,
             const GenerateContext &generateContext,
             std::vector<float>& motionFrames,
             const UsdGeomBasisCurves &curves,
             const scene_rdl2::rdl2::Layer *rdlLayer,
             const CommonAttrs& commonAttrs,
             float radiusMult,
             float baseWidthFactor,
             float tipWidthFactor,
             int curvedMotionBlurSampleCount);

} // namespace usd
} // namespace moonshine

