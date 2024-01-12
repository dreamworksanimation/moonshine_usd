// Copyright 2023-2024 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#include "Curves.h"

#include <moonshine/geometry/data/PrimitiveUserData.h>
#include <moonray/rendering/geom/Api.h>
#include <moonray/rendering/geom/ProceduralLeaf.h>
#include <moonray/rendering/shading/ShadingUtil.h>
#include <scene_rdl2/common/platform/Platform.h>
#include <scene_rdl2/scene/rdl2/rdl2.h>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdSkel/bindingAPI.h>
#include <pxr/usd/usdSkel/cache.h>
#include <pxr/usd/usdSkel/root.h>
#include <pxr/usd/usdSkel/skeletonQuery.h>
#include <pxr/usd/usdSkel/skinningQuery.h>

#include <string>
#include <random>
#include <functional>

using namespace moonray;
using namespace moonray::geom;
using namespace moonray::shading;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

moonray::geom::Curves::Type
getCurvesType(const UsdGeomBasisCurves& curves) {
    TfToken usdType;
    curves.GetTypeAttr().Get(&usdType);
    if (usdType == "linear") {
        return moonray::geom::Curves::Type::LINEAR;
    } else {
        TfToken usdBasis;
        curves.GetBasisAttr().Get(&usdBasis);
        if (usdBasis == "bspline") {
            return moonray::geom::Curves::Type::BSPLINE;
        } else if (usdBasis == "bezier") {
            return moonray::geom::Curves::Type::BEZIER;
        } else {
            return moonray::geom::Curves::Type::UNKNOWN;
        }
    }
}

bool
isCurvePinned(const VertexBuffer<Vec3fa, InterleavedTraits>& vertices,
              const size_t vertIndex,
              const size_t numVertices)
{
    if (numVertices < 6) {
        return false;
    } else {
        const Vec3fa& firstVert0 = vertices(vertIndex + 0, 0);
        const Vec3fa& firstVert1 = vertices(vertIndex + 1, 0);
        const Vec3fa& firstVert2 = vertices(vertIndex + 2, 0);
        const Vec3fa& lastVert0 = vertices(vertIndex + numVertices - 3, 0);
        const Vec3fa& lastVert1 = vertices(vertIndex + numVertices - 2, 0);
        const Vec3fa& lastVert2 = vertices(vertIndex + numVertices - 1, 0);

        return scene_rdl2::math::isEqual(firstVert0.x, firstVert1.x) && scene_rdl2::math::isEqual(firstVert1.x, firstVert2.x) &&
               scene_rdl2::math::isEqual(firstVert0.y, firstVert1.y) && scene_rdl2::math::isEqual(firstVert1.y, firstVert2.y) &&
               scene_rdl2::math::isEqual(firstVert0.z, firstVert1.z) && scene_rdl2::math::isEqual(firstVert1.z, firstVert2.z) &&
               scene_rdl2::math::isEqual(lastVert0.x, lastVert1.x) && scene_rdl2::math::isEqual(lastVert1.x, lastVert2.x) &&
               scene_rdl2::math::isEqual(lastVert0.y, lastVert1.y) && scene_rdl2::math::isEqual(lastVert1.y, lastVert2.y) &&
               scene_rdl2::math::isEqual(lastVert0.z, lastVert1.z) && scene_rdl2::math::isEqual(lastVert1.z, lastVert2.z);
    }
}

void
applyRadiusFactors(std::vector<float>& radius,
                   const VertexBuffer<Vec3fa, InterleavedTraits>& vertices,
                   const ::moonray::geom::Curves::CurvesVertexCount& curvesVertexCount,
                   size_t curvesCount,
                   float radiusMult,
                   float baseWidthFactor,
                   float tipWidthFactor)
{
    if (!::scene_rdl2::math::isEqual(radiusMult, 1.0f) ||
        !::scene_rdl2::math::isEqual(baseWidthFactor, 1.0f) ||
        !::scene_rdl2::math::isEqual(tipWidthFactor, 1.0f)) {

        size_t vertIndex = 0;
        for (size_t i = 0; i < curvesCount; ++i) {
            size_t numV = curvesVertexCount[i];

            // Check if the curve is pinned by comparing the first three
            // vertices for equality and the last three.  Even though Usd
            // has a wrap type "pinned" that we could check, the curves
            // written out of Houdini don't use this and have extra vertices
            // instead with wrap type set to "nonperiodic".  Therefore comparing
            // the vertices is the only way we can currently tell if the curve
            // is pinned.
            const bool pinned = isCurvePinned(vertices, vertIndex, numV);

            if (numV == 1) {
                radius[vertIndex] *= radiusMult * (baseWidthFactor * 0.5f + tipWidthFactor * 0.5f);
                ++vertIndex;
            } else {
                for (size_t v = 0; v < numV; ++v) {
                    float curveT;
                    if (pinned && v < 3) {
                        curveT = 0.0f;
                    } else if (pinned && v >= (numV - 3)) {
                        curveT = 1.0f;
                    } else {
                        curveT = (static_cast<float>(pinned ? v - 2 : v)) /
                                 (static_cast<float>(pinned ?  numV - 5 : numV - 1));
                    }
                    radius[vertIndex] *= radiusMult * (baseWidthFactor * (1.0 - curveT) + tipWidthFactor * curveT);
                    ++vertIndex;
                }
            }
        }
    }
}

std::vector<float>
curvesExtractRadius(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                    const UsdGeomBasisCurves& curves,
                    const size_t curvesCount,
                    const size_t vertexCount,
                    const PolygonMesh::FaceVertexCount& curveVertexCounts,
                    const VertexBuffer<Vec3fa, InterleavedTraits>& vertices,
                    std::vector<float>& motionFrames,
                    float radiusMult,
                    float baseWidthFactor,
                    float tipWidthFactor)
{
    std::vector<float> radius(vertexCount, 1.0f);
    VtArray<float> widths;
    float frame = motionFrames[0];
    curves.GetWidthsAttr().Get(&widths, frame);
    if (!widths.empty()) {
        TfToken widthsInterpolation = curves.GetWidthsInterpolation();
        if (widthsInterpolation == UsdGeomTokens->constant) {
            float r = 0.5f * widths[0];
            for (size_t i = 0; i < radius.size(); ++i) {
                radius[i] *= r;
            }
        } else if (widthsInterpolation == UsdGeomTokens->uniform) {
            size_t offset = 0;
            for (size_t i = 0; i < curvesCount; ++i) {
                float r = 0.5f * widths[i];
                for (size_t j = 0; j < curveVertexCounts[i]; ++j) {
                    radius[offset++] *= r;
                }
            }
        } else if (widthsInterpolation == UsdGeomTokens->faceVarying ||
                   widthsInterpolation == UsdGeomTokens->varying ||
                   widthsInterpolation == UsdGeomTokens->vertex) {
            for (size_t i = 0 ; i < vertexCount; ++i) {
                radius[i] *= 0.5f * widths[i];
            }
        }
    }

    // optional pscale factor
    moonray::shading::TypedAttributeKey<float> pscaleKey("pscale");
    std::vector< std::vector<float> > pscaleValues;
    AttributeRate rate;
    if (::moonshine::usd::getPrimvarVector<float, float, UsdGeomBasisCurves>(rdlGeometry,
                                                                             curves,
                                                                             pscaleKey,
                                                                             motionFrames,
                                                                             rate,
                                                                             pscaleValues)) {
        if (rate == RATE_CONSTANT) {
            float pscale = pscaleValues[0][0];
            for (size_t i = 0; i < radius.size(); ++i) {
                radius[i] *= pscale;
            }
        } else if (rate == RATE_UNIFORM) {
            size_t offset = 0;
            for (size_t i = 0; i < curvesCount; ++i) {
                float pscale = pscaleValues[0][i];
                for (size_t j = 0; j < curveVertexCounts[i]; ++j) {
                    radius[offset++] *= pscale;
                }
            }
        } else if (rate == RATE_VARYING ||
                   rate == RATE_FACE_VARYING ||
                   rate == RATE_VERTEX) {
            for (size_t i = 0 ; i < vertexCount; ++i) {
                radius[i] *= pscaleValues[0][i];
            }
        }
    }

    // Apply radius multiplier and base/tip width factors
    applyRadiusFactors(radius,
                       vertices,
                       curveVertexCounts,
                       curvesCount,
                       radiusMult,
                       baseWidthFactor,
                       tipWidthFactor);

    return radius;
}

void
addScatterTag(const scene_rdl2::rdl2::Geometry *rdlGeometry,
              const GenerateContext& generateContext,
              const size_t curvesCount,
              PrimitiveAttributeTable& primitiveAttributeTable)
{
    if (generateContext.requestAttribute(moonray::shading::StandardAttributes::sScatterTag)) {
        // Seed with geometry name so each primitive has a unique random
        // distribution associated with its curves
        std::ranlux24 generator(std::hash<std::string>{}(rdlGeometry->getName()));
        std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

        // add per curve scatter tag
        std::vector<float> scatterTag;
        scatterTag.reserve(curvesCount);
        for (size_t curveIdx = 0; curveIdx < curvesCount; ++curveIdx) {
            scatterTag.emplace_back(distribution(generator));
        }
        // scatter_tag added to primitive attributes table (separate from AbcCurves attributes)
        primitiveAttributeTable.addAttribute(moonray::shading::StandardAttributes::sScatterTag,
                                             RATE_UNIFORM,
                                             std::move(scatterTag));
    }
}


} // end anonymous namespace

namespace moonshine {
namespace usd {

std::unique_ptr<Primitive>
createCurves(const scene_rdl2::rdl2::Geometry *rdlGeometry,
             const GenerateContext &generateContext,
             std::vector<float>& motionFrames,
             const UsdGeomBasisCurves& curves,
             const scene_rdl2::rdl2::Layer *rdlLayer,
             const CommonAttrs& commonAttrs,
             float radiusMult,
             float baseWidthFactor,
             float tipWidthFactor,
             int curvedMotionBlurSampleCount)
{
    const float currentFrame = commonAttrs.useEvaluationFrame ?
                               commonAttrs.evaluationFrame :
                               generateContext.getCurrentFrame();

    // motionFrames may be updated here, depending on discovered mbType
    const scene_rdl2::rdl2::MotionBlurType mbType =
        getMotionBlurType<UsdGeomBasisCurves>(curves,
                                              commonAttrs.motionBlurType,
                                              motionFrames,
                                              currentFrame);

    PrimitiveAttributeTable primitiveAttributeTable;

    const std::string curvesName = curves.GetPrim().GetName();
    int assignmentId;
    if (!getAssignmentId(rdlLayer,
                         rdlGeometry,
                         curvesName,
                         assignmentId)) {
        return nullptr;
    }

    moonray::geom::LayerAssignmentId layerAssignmentId =
        moonray::geom::LayerAssignmentId(assignmentId);

    moonray::geom::Curves::Type curvesType = getCurvesType(curves);
    if (curvesType == Curves::Type::UNKNOWN) {
        rdlGeometry->warn("Curves primitive ", curvesName,
            " contains unsupported curves type. "
            "Fall back to linear line segments");
        curvesType = moonray::geom::Curves::Type::LINEAR;
    }

    // Set the vert per face count
    VtArray<int> tmp;
    curves.GetCurveVertexCountsAttr().Get(&tmp, currentFrame);
    PolygonMesh::FaceVertexCount curveVertexCounts(tmp.begin(), tmp.end());


    VertexBuffer<Vec3fa, InterleavedTraits> vertices =
        getVertices<UsdGeomBasisCurves>(rdlGeometry,
                                        curves,
                                        primitiveAttributeTable,
                                        mbType,
                                        motionFrames);

    const size_t curvesCount = curveVertexCounts.size();
    const size_t vertexCount = vertices.size();

    std::vector<float> radius = curvesExtractRadius(rdlGeometry,
                                                    curves,
                                                    curvesCount,
                                                    vertexCount,
                                                    curveVertexCounts,
                                                    vertices,
                                                    motionFrames,
                                                    radiusMult,
                                                    baseWidthFactor,
                                                    tipWidthFactor);

    // Copy radius values to w-components
    size_t samplesCount = vertices.get_time_steps();
    for (size_t s = 0; s < samplesCount; s++) {
        for (size_t i = 0; i < vertexCount; i++) {
            vertices(i, s).w = radius[i];
        }
    }

    // check if curves are valid - if not, early-out
    if (vertices.empty() || curveVertexCounts.empty()) {
        return nullptr;
    }

    // Add any primitive attribute requested for shading
    for (const auto& key : generateContext.getRequestedAttributes()) {
        addPrimvar(rdlGeometry,
                   curves,
                   key,
                   primitiveAttributeTable,
                   motionFrames);
    }

    addScatterTag(rdlGeometry,
                  generateContext,
                  curvesCount,
                  primitiveAttributeTable);

    // Populate the primitive attribute table with any user data
    const scene_rdl2::rdl2::PrimitiveAttributeFrame primitiveAttributeFrame = commonAttrs.primitiveAttributeFrame;
    bool useFirstFrame = (primitiveAttributeFrame != scene_rdl2::rdl2::PrimitiveAttributeFrame::SECOND_MOTION_STEP);
    bool useSecondFrame = (primitiveAttributeFrame != scene_rdl2::rdl2::PrimitiveAttributeFrame::FIRST_MOTION_STEP);
    geometry::processUserData(commonAttrs.primitiveAttributes,
                              &commonAttrs.partList,
                              curvesName,
                              useFirstFrame,
                              useSecondFrame,
                              primitiveAttributeTable);

    // Add explicit shading primitive attribute if it's enabled
    if (commonAttrs.explicitShading &&
        !addExplicitShading(rdlGeometry, primitiveAttributeTable)) {

        return nullptr;
    }

    std::unique_ptr<Curves> primitive =
        moonray::geom::createCurves(curvesType,
                                    commonAttrs.curvesSubType,
                                    commonAttrs.tessellationRate,
                                    std::move(curveVertexCounts),
                                    std::move(vertices),
                                    std::move(layerAssignmentId),
                                    std::move(primitiveAttributeTable));

    if (primitive) {
        primitive->setName(curvesName);
        primitive->setCurvedMotionBlurSampleCount(curvedMotionBlurSampleCount);
    }

    return std::move(primitive);
}

} // namespace usd
} // namespace moonshine

