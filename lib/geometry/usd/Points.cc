// Copyright 2023 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#include "Points.h"

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

using namespace moonray;
using namespace moonray::geom;
using namespace moonray::shading;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

moonray::geom::Points::RadiusBuffer
pointsExtractRadius(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                    const UsdGeomPoints& points,
                    const size_t vertexCount,
                    const float pointScale,
                    std::vector<float>& motionFrames)
{
    moonray::geom::Points::RadiusBuffer radius(vertexCount, pointScale);

    VtArray<float> widths;
    float frame = motionFrames[0];
    points.GetWidthsAttr().Get(&widths, frame);
    if (!widths.empty()) {
        TfToken widthsInterpolation = points.GetWidthsInterpolation();
        if (widthsInterpolation == UsdGeomTokens->constant ||
            widthsInterpolation == UsdGeomTokens->uniform) {
            float r = 0.5f * widths[0];
            for (size_t i = 0; i < radius.size(); ++i) {
                radius[i] *= r;
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
    if (moonshine::usd::getPrimvarVector<float, float, UsdGeomPoints>(rdlGeometry,
                                                                      points,
                                                                      pscaleKey,
                                                                      motionFrames,
                                                                      rate,
                                                                      pscaleValues)) {
        if (rate == RATE_CONSTANT || rate == RATE_UNIFORM) {
            float pscale = pscaleValues[0][0];
            for (size_t i = 0; i < radius.size(); ++i) {
                radius[i] *= pscale;
            }
        } else if (rate == RATE_VARYING ||
                   rate == RATE_FACE_VARYING ||
                   rate == RATE_VERTEX) {
            for (size_t i = 0 ; i < vertexCount; ++i) {
                radius[i] *= pscaleValues[0][i];
            }
        }
    }

    return radius;
}

void
pointsExtractTextureUV(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                       const UsdGeomPoints& points,
                       PrimitiveAttributeTable& primitiveAttributeTable,
                       std::vector<float>& motionFrames)
{
    std::vector< std::vector<scene_rdl2::math::Vec2f> > textureUV;
    AttributeRate rate;
    if (!moonshine::usd::getPrimvarVector<scene_rdl2::math::Vec2f, GfVec2f, UsdGeomPoints>(rdlGeometry,
                                                                                           points,
                                                                                           StandardAttributes::sUv,
                                                                                           motionFrames,
                                                                                           rate,
                                                                                           textureUV)) {
        return;
    }


    primitiveAttributeTable.addAttribute(StandardAttributes::sUv,
                                         rate,
                                         std::move(textureUV));
}

void
pointsExtractNormal(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                    const UsdGeomPoints& points,
                    PrimitiveAttributeTable& primitiveAttributeTable,
                    std::vector<float>& motionFrames)
{
    std::vector< std::vector<scene_rdl2::math::Vec3f> > data;
    AttributeRate rate;
    if (!moonshine::usd::getPrimvarVector<scene_rdl2::math::Vec3f, GfVec3f, UsdGeomPoints>(rdlGeometry,
                                                                               points,
                                                                               StandardAttributes::sNormal,
                                                                               motionFrames,
                                                                               rate,
                                                                               data)) {
        return;
    }

    primitiveAttributeTable.addAttribute(StandardAttributes::sNormal,
                                         rate,
                                         std::move(data));
}

void
pointsExtractDerivatives(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                         const UsdGeomPoints& points,
                         PrimitiveAttributeTable& primitiveAttributeTable,
                         std::vector<float>& motionFrames)
{
    AttributeRate dPdsRate;
    std::vector< std::vector<scene_rdl2::math::Vec3f> > dPdsData;
    if (!moonshine::usd::getPrimvarVector<scene_rdl2::math::Vec3f, GfVec3f, UsdGeomPoints>(rdlGeometry,
                                                                               points,
                                                                               StandardAttributes::sdPds,
                                                                               motionFrames,
                                                                               dPdsRate,
                                                                               dPdsData)) {
        return;
    }

    AttributeRate dPdtRate;
    std::vector< std::vector<scene_rdl2::math::Vec3f> > dPdtData;
    if (!moonshine::usd::getPrimvarVector<scene_rdl2::math::Vec3f, GfVec3f, UsdGeomPoints>(rdlGeometry,
                                                                               points,
                                                                               StandardAttributes::sdPdt,
                                                                               motionFrames,
                                                                               dPdtRate,
                                                                               dPdtData)) {
        return;
    }

    // Add data to the primitive attribute table only if both
    // dPds and dPdt data exists on the points
    primitiveAttributeTable.addAttribute(StandardAttributes::sdPds,
                                         dPdsRate,
                                         std::move(dPdsData));

    primitiveAttributeTable.addAttribute(StandardAttributes::sdPdt,
                                         dPdtRate,
                                         std::move(dPdtData));
}

} // end anonymous namespace

namespace moonshine {
namespace usd {

std::unique_ptr<Primitive>
createPoints(const scene_rdl2::rdl2::Geometry *rdlGeometry,
             const GenerateContext &generateContext,
             std::vector<float>& motionFrames,
             const UsdGeomPoints& points,
             const scene_rdl2::rdl2::Layer *rdlLayer,
             const CommonAttrs& commonAttrs,
             float pointScale,
             int curvedMotionBlurSampleCount)
{
    const float currentFrame = commonAttrs.useEvaluationFrame ?
                               commonAttrs.evaluationFrame :
                               generateContext.getCurrentFrame();

    // motionFrames may be updated here, depending on discovered mbType
    const scene_rdl2::rdl2::MotionBlurType mbType =
        getMotionBlurType<UsdGeomPoints>(points,
                                         commonAttrs.motionBlurType,
                                         motionFrames,
                                         currentFrame);

    PrimitiveAttributeTable primitiveAttributeTable;

    const std::string pointsName = points.GetPrim().GetName();
    int assignmentId;
    if (!getAssignmentId(rdlLayer,
                         rdlGeometry,
                         pointsName,
                         assignmentId)) {
        return nullptr;
    }

    moonray::geom::LayerAssignmentId layerAssignmentId =
        moonray::geom::LayerAssignmentId(assignmentId);

    VertexBuffer<Vec3fa, InterleavedTraits> vertices =
        getVertices<UsdGeomPoints>(rdlGeometry,
                                   points,
                                   primitiveAttributeTable,
                                   mbType,
                                   motionFrames);

    const size_t vertexCount = vertices.size();

    VertexBuffer<scene_rdl2::math::Vec3f, InterleavedTraits> positions(vertexCount, 1);
    for (size_t i = 0; i < vertexCount; ++i) {
        positions(i) = scene_rdl2::math::Vec3f(vertices(i)[0],
                                   vertices(i)[1],
                                   vertices(i)[2]);
    }

    moonray::geom::Points::RadiusBuffer radius =
        pointsExtractRadius(rdlGeometry,
                            points,
                            vertexCount,
                            pointScale,
                            motionFrames);

    // check if curves are valid - if not, early-out
    if (vertices.empty() || radius.empty()) {
        return nullptr;
    }

    // optional texture uv
    pointsExtractTextureUV(rdlGeometry,
                           points,
                           primitiveAttributeTable,
                           motionFrames);

    // optional normals
    pointsExtractNormal(rdlGeometry,
                        points,
                        primitiveAttributeTable,
                        motionFrames);

    // optional derivatives
    pointsExtractDerivatives(rdlGeometry,
                             points,
                             primitiveAttributeTable,
                             motionFrames);

    // Add any primitive attribute requested for shading
    for (const auto& key : generateContext.getRequestedAttributes()) {
        addPrimvar(rdlGeometry,
                   points,
                   key,
                   primitiveAttributeTable,
                   motionFrames);
    }

    // Populate the primitive attribute table with any user data
    const scene_rdl2::rdl2::PrimitiveAttributeFrame primitiveAttributeFrame = commonAttrs.primitiveAttributeFrame;
    bool useFirstFrame = (primitiveAttributeFrame != scene_rdl2::rdl2::PrimitiveAttributeFrame::SECOND_MOTION_STEP);
    bool useSecondFrame = (primitiveAttributeFrame != scene_rdl2::rdl2::PrimitiveAttributeFrame::FIRST_MOTION_STEP);
    geometry::processUserData(commonAttrs.primitiveAttributes,
                              &commonAttrs.partList,
                              pointsName,
                              useFirstFrame,
                              useSecondFrame,
                              primitiveAttributeTable);

    // Add explicit shading primitive attribute if it's enabled
    if (commonAttrs.explicitShading &&
        !addExplicitShading(rdlGeometry, primitiveAttributeTable)) {

        return nullptr;
    }


    std::unique_ptr<Points> primitive =
        createPoints(std::move(positions),
                     std::move(radius),
                     std::move(layerAssignmentId),
                     std::move(primitiveAttributeTable));
    if (primitive) {
        primitive->setName(pointsName);
        primitive->setCurvedMotionBlurSampleCount(curvedMotionBlurSampleCount);
    }
    return std::move(primitive);
}

} // namespace usd
} // namespace moonshine

