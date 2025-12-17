// Copyright 2023-2024 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#pragma once

#include <moonray/rendering/geom/Api.h>
#include <moonray/rendering/geom/ProceduralContext.h>
#include <moonray/rendering/geom/ProceduralLeaf.h>
#include <scene_rdl2/scene/rdl2/rdl2.h>
#include <scene_rdl2/common/platform/Platform.h>

#include <pxr/pxr.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/points.h>
#include <pxr/usd/usdGeom/pointInstancer.h>
#include <pxr/usd/usdGeom/basisCurves.h>
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

using namespace moonray;
using namespace moonray::geom;
using namespace moonray::shading;

PXR_NAMESPACE_USING_DIRECTIVE

namespace moonshine {
namespace usd {

struct CommonAttrs {
    bool useEvaluationFrame;
    float evaluationFrame;
    bool useRotationMotionBlur;
    scene_rdl2::rdl2::MotionBlurType motionBlurType;
    uint32_t curvedMotionBlurSampleCount;
    std::vector<std::string> partList;
    std::vector<scene_rdl2::rdl2::SceneObject*> primitiveAttributes;
    scene_rdl2::rdl2::PrimitiveAttributeFrame primitiveAttributeFrame;
    Curves::SubType curvesSubType;
    int tessellationRate;
    bool explicitShading;

    CommonAttrs()
        : useEvaluationFrame(false)
        , evaluationFrame(0.0f)
        , useRotationMotionBlur(false)
        , motionBlurType(scene_rdl2::rdl2::MotionBlurType::STATIC)
        , curvedMotionBlurSampleCount(10)
        , primitiveAttributeFrame(scene_rdl2::rdl2::PrimitiveAttributeFrame::BOTH_MOTION_STEPS)
        , curvesSubType(Curves::SubType::UNKNOWN)
        , tessellationRate(4)
        , explicitShading(false)
    {}
};

template<typename GeomType>
bool
isTopologyHomogeneous(const GeomType& usdGeometry,
                      const std::vector<float>& motionFrames);

template<typename GeomType>
scene_rdl2::rdl2::MotionBlurType
getMotionBlurType(const GeomType& usdGeometry,
                  scene_rdl2::rdl2::MotionBlurType requestedMbType,
                  std::vector<float>& motionFrames,
                  float currentFrame);

template<typename GeomType>
void
addPrimvar(const scene_rdl2::rdl2::Geometry *rdlGeometry,
           const GeomType& usdGeometry,
           const AttributeKey& rdlAttrKey,
           PrimitiveAttributeTable& table,
           const std::vector<float>& motionFrames);

void
addPointInstancerPrimvar(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                         const UsdGeomPointInstancer& pointInstancer,
                         const AttributeKey& rdlAttrKey,
                         PrimitiveAttributeTable& table,
                         const std::vector<float>& motionFrames);

void
addPointInstancerUVs(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                     const UsdGeomPointInstancer& pointInstancer,
                     PrimitiveAttributeTable& table,
                     const std::vector<float>& motionFrames);

void
addPointInstancerNormals(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                         const UsdGeomPointInstancer& pointInstancer,
                         PrimitiveAttributeTable& table,
                         const std::vector<float>& motionFrames);

void
addPointInstancerDerivatives(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                             const UsdGeomPointInstancer& pointInstancer,
                             PrimitiveAttributeTable& table,
                             const std::vector<float>& motionFrames);

template<typename UsdType>
void
setStaticPrimvar(const UsdGeomPrimvar& primvar,
                 PrimitiveAttributeTable& table,
                 float frame,
                 const AttributeKey& rdlAttrKey,
                 AttributeRate attrRate)
{
    std::vector<UsdType> result;

    if (attrRate == RATE_CONSTANT) {
        UsdType data;
        if (primvar.Get(&data, frame)) {
            result.push_back(data);
        }
    } else if (attrRate == RATE_UNIFORM) {
        VtArray<UsdType> data;
        if (primvar.Get(&data, frame)) {
            result.assign(std::begin(data), std::end(data));
        }
    } else {
        MNRY_ASSERT(false, "static primitive attribute only support"
                          " constant and uniform rates");
    }

    table.addAttribute(TypedAttributeKey<UsdType>(primvar.GetPrimvarName().GetString()),
                       attrRate,
                       std::move(result));
}

void
finline emplaceOnePrimvar(std::vector<float>& result,
                          const float& item)
{
    result.emplace_back(item);
}

void
finline emplaceOnePrimvar(std::vector<Vec2f>& result,
                          const GfVec2f& item)
{
    result.emplace_back(
        item[0],
        item[1]
    );
}

void
finline emplaceOnePrimvar(std::vector<Vec3f>& result,
                          const GfVec3f& item)
{
    result.emplace_back(
        item[0],
        item[1],
        item[2]
    );
}

void
finline emplaceOnePrimvar(std::vector<scene_rdl2::math::Color>& result,
                          const GfVec3f& item)
{
    result.emplace_back(
        item[0],
        item[1],
        item[2]
    );
}

void
finline emplaceOnePrimvar(std::vector<scene_rdl2::math::Color4>& result,
                          const GfVec4f& item)
{
    result.emplace_back(
        item[0],
        item[1],
        item[2],
        item[3]
    );
}

void
finline emplaceOnePrimvar(std::vector<scene_rdl2::math::Mat4f>& result,
                          const GfMatrix4d& item)
{
    result.emplace_back(
        item[0][0], item[0][1], item[0][2], item[0][3],
        item[1][0], item[1][1], item[1][2], item[1][3],
        item[2][0], item[2][1], item[2][2], item[2][3],
        item[3][0], item[3][1], item[3][2], item[3][3]
    );
}

template<typename GeomType>
bool
isLeftHanded(const GeomType& usdGeometry);

template<typename GeomType>
void
getPrimVertexCounts(const GeomType& usdGeometry,
                    VtArray<int>& primVertexCounts,
                    float frame);

bool
finline getPrimvarRate(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                       const UsdGeomPrimvar& primvar,
                       const AttributeKey& rdlAttrKey,
                       AttributeRate& attrRate)
{
    const TfToken interpolation = primvar.GetInterpolation();
    attrRate = RATE_UNKNOWN;
    if (interpolation == UsdGeomTokens->vertex) {
        // default
        attrRate = RATE_VERTEX;
    } else if (interpolation == UsdGeomTokens->faceVarying) {
        attrRate = RATE_FACE_VARYING;
    } else if (interpolation == UsdGeomTokens->uniform) {
        attrRate = RATE_UNIFORM;
    } else if (interpolation == UsdGeomTokens->varying) {
        attrRate = RATE_VARYING;
    } else if (interpolation == UsdGeomTokens->constant) {
        attrRate = RATE_CONSTANT;
    } else {
        return false;
    }

    return true;
}

bool
primvarTypeIsCorrect(const UsdGeomPrimvar& primvar,
                     const AttributeKey& rdlAttrKey);


template<typename GeomType>
bool
getPrimvar(const scene_rdl2::rdl2::Geometry *rdlGeometry,
           const GeomType& usdGeometry,
           const AttributeKey& rdlAttrKey,
           UsdGeomPrimvar& primvar,
           const std::vector<float>& motionFrames)
{
    const UsdGeomPrimvarsAPI pvAPI(usdGeometry.GetPrim());
    if (!pvAPI.HasPrimvar(TfToken(rdlAttrKey.getName()))) {
        return false;
    }

    primvar = pvAPI.GetPrimvar(TfToken(rdlAttrKey.getName()));
    if (!primvarTypeIsCorrect(primvar, rdlAttrKey)) {
        return false;
    }

    VtValue data;
    if (!primvar.Get(&data, motionFrames[0])) {
        return false;
    }

    return true;
}

template<typename MoonrayType, typename UsdType, typename GeomType>
bool
getPrimvarVectorData(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                     const GeomType& usdGeometry,
                     const UsdGeomPrimvar& primvar,
                     const AttributeKey& rdlAttrKey,
                     const std::vector<float>& motionFrames,
                     AttributeRate attrRate,
                     std::vector<std::vector<MoonrayType>>& result)
{
    float frame = motionFrames.back();

    // Get the vert per face count
    VtArray<int> faceVertexCounts;
    getPrimVertexCounts<GeomType>(usdGeometry,
                                  faceVertexCounts,
                                  frame);
    VtArray <GfVec3f> points;
    usdGeometry.GetPointsAttr().Get(&points, frame);
    const uint32_t faceCount = faceVertexCounts.size();
    const uint32_t vertexCount = points.size();

    std::vector<float> usableMotionFrames;
    if (attrRate == RATE_CONSTANT || !isTopologyHomogeneous<GeomType>(usdGeometry, motionFrames)) {
        usableMotionFrames.clear();
        usableMotionFrames.push_back(motionFrames.back());
    } else {
        usableMotionFrames = motionFrames;
    }

    result.resize(usableMotionFrames.size());

    const bool isLH = isLeftHanded(usdGeometry);

    for (size_t t = 0; t < usableMotionFrames.size(); ++t) {
        frame = usableMotionFrames[t];
        if (attrRate == RATE_CONSTANT) {
            VtValue data;
            if (primvar.Get(&data, frame)) {
                if (data.IsHolding<UsdType>()) {
                    // TODO: investigate - it's possible this never happens
                    emplaceOnePrimvar(result[t], data.Get<UsdType>());
                    continue;
                } else if (data.IsHolding<VtArray<UsdType>>()) {
                    emplaceOnePrimvar(result[t], data.Get<VtArray<UsdType>>()[0]);
                    continue;
                }
            }

            return false;
        }

        VtArray<UsdType> data;
        if (!primvar.Get(&data, frame)) {
            return false;
        }

        if (attrRate == RATE_FACE_VARYING) {
            // Check if the attribute is indexed
            bool isIndexed = false;
            VtArray<int> primvarIndices;
            if (primvar.GetIndicesAttr().HasValue()) {
                primvar.GetIndicesAttr().Get(&primvarIndices, frame);
                isIndexed = true;
            }

            // 3 or 4 vertices per face
            result.reserve(4 * faceCount);
            uint32_t indexOffset = 0;
            for (uint32_t fIndex = 0; fIndex < faceCount; ++fIndex) {
                uint32_t faceVertexCount = faceVertexCounts[fIndex];
                for (uint32_t vIndex = 0; vIndex < faceVertexCount; ++vIndex) {
                    // reverse winding order for left handed geometry
                    uint32_t realVIndex = isLH ? (faceVertexCount - vIndex - 1) : vIndex;
                    const uint32_t primvarIndex = (isIndexed) ? primvarIndices[indexOffset + realVIndex] :
                                                                indexOffset + realVIndex;
                    emplaceOnePrimvar(result[t],
                                      data[primvarIndex]);
                }
                indexOffset += faceVertexCount;
            }
        } else if (attrRate == RATE_VARYING) {
            result.reserve(vertexCount);
            for (uint32_t vId = 0; vId < vertexCount; ++vId) {
                emplaceOnePrimvar(result[t], data[vId]);
            }
        } else if (attrRate == RATE_VERTEX) {
            result.reserve(vertexCount);
            for (uint32_t vId = 0; vId < vertexCount; ++vId) {
                emplaceOnePrimvar(result[t], data[vId]);
            }
        } else if (attrRate == RATE_UNIFORM) {
            result.reserve(faceCount);
            for (uint32_t fIndex = 0; fIndex < faceCount; ++fIndex) {
                emplaceOnePrimvar(result[t], data[fIndex]);
            }
        }
    }

    return true;
}

template<typename MoonrayType, typename UsdType>
bool
getPointInstancerPrimvarVectorData(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                                   const UsdGeomPrimvar& primvar,
                                   const AttributeKey& rdlAttrKey,
                                   const std::vector<float>& motionFrames,
                                   AttributeRate attrRate,
                                   std::vector<MoonrayType>& result)
{
    const float frame = motionFrames.back();
    if (attrRate == RATE_CONSTANT) {
        VtValue data;
        if (!primvar.Get(&data, frame)) {
            return false;
        }

        if (data.IsHolding<UsdType>()) {
            // TODO: investigate - it's possible this never happens
            emplaceOnePrimvar(result, data.Get<UsdType>());
        } else if (data.IsHolding<VtArray<UsdType>>()) {
            emplaceOnePrimvar(result, data.Get<VtArray<UsdType>>()[0]);
        }
    } else {
        VtArray<UsdType> data;
        if (!primvar.Get(&data, frame)) {
            return false;
        }

        if (attrRate == RATE_VERTEX || attrRate == RATE_VARYING) {
            for (uint32_t vId = 0; vId < data.size(); ++vId) {
                emplaceOnePrimvar(result, data[vId]);
            }
        }
    }

    return true;
}

template<typename MoonrayType, typename UsdType, typename GeomType>
bool
getPrimvarVector(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                 const GeomType& usdGeometry,
                 const AttributeKey& rdlAttrKey,
                 const std::vector<float>& motionFrames,
                 AttributeRate& outputAttrRate,
                 std::vector<std::vector<MoonrayType>>& outputVector)
{
    UsdGeomPrimvar primvar;
    if (!getPrimvar<GeomType>(rdlGeometry, usdGeometry, rdlAttrKey, primvar, motionFrames)) {
        return false;
    }

    if (!getPrimvarRate(rdlGeometry, primvar, rdlAttrKey, outputAttrRate)) {
        return false;
    }

    return getPrimvarVectorData<MoonrayType, UsdType, GeomType>(rdlGeometry,
                                                                usdGeometry,
                                                                primvar,
                                                                rdlAttrKey,
                                                                motionFrames,
                                                                outputAttrRate,
                                                                outputVector);
}

template<typename MoonrayType, typename UsdType, typename GeomType>
void
setPrimvar(const scene_rdl2::rdl2::Geometry *rdlGeometry,
           const GeomType& usdGeometry,
           const UsdGeomPrimvar& primvar,
           PrimitiveAttributeTable& table,
           const std::vector<float>& motionFrames,
           const AttributeKey& rdlAttrKey,
           AttributeRate attrRate)
{
    std::vector< std::vector<MoonrayType> > result;
    getPrimvarVectorData<MoonrayType, UsdType, GeomType>(rdlGeometry,
                                                         usdGeometry,
                                                         primvar,
                                                         rdlAttrKey,
                                                         motionFrames,
                                                         attrRate,
                                                         result);

    table.addAttribute(TypedAttributeKey<MoonrayType>(primvar.GetPrimvarName().GetString()),
                       attrRate,
                       std::move(result));
}

template<typename MoonrayType, typename UsdType>
void
setPointInstancerPrimvar(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                         const UsdGeomPrimvar& primvar,
                         PrimitiveAttributeTable& table,
                         const std::vector<float>& motionFrames,
                         const AttributeKey& rdlAttrKey,
                         AttributeRate attrRate)
{
    std::vector<MoonrayType> result;
    getPointInstancerPrimvarVectorData<MoonrayType, UsdType>(rdlGeometry,
                                                             primvar,
                                                             rdlAttrKey,
                                                             motionFrames,
                                                             attrRate,
                                                             result);

    table.addAttribute(TypedAttributeKey<MoonrayType>(primvar.GetPrimvarName().GetString()),
                       attrRate,
                       std::move(result));
}

template<typename GeomType>
VertexBuffer<Vec3fa, InterleavedTraits>
getVertices(const scene_rdl2::rdl2::Geometry *rdlGeometry,
            const GeomType& usdGeometry,
            PrimitiveAttributeTable& table,
            scene_rdl2::rdl2::MotionBlurType mbType,
            const std::vector<float>& motionFrames);

// Would like a builtin token for skel, but can't find one avail in UsdSkelTokens
const TfToken SKEL("skel");
bool
finline expandSkelMask(const UsdRelationship& rel)
{
    return rel.GetNamespace() == SKEL;
}

} // namespace usd
} // namespace moonshine

