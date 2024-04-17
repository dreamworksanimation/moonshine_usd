// Copyright 2023-2024 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#include "Common.h"

#include <scene_rdl2/scene/rdl2/rdl2.h>
#include <scene_rdl2/common/platform/Platform.h>
#include <moonray/rendering/geom/Api.h>
#include <moonray/rendering/geom/ProceduralContext.h>

#include <pxr/pxr.h>
#include <pxr/usd/usd/stageCache.h>

using namespace moonray;
using namespace moonray::geom;

PXR_NAMESPACE_USING_DIRECTIVE

namespace moonshine {
namespace usd {

template<typename GeomType>
bool
isTopologyHomogeneous(const GeomType& usdGeometry,
                      const std::vector<float>& motionFrames)
{
    if (motionFrames.size() < 2) {
        return true;
    }

    VtValue checker;
    usdGeometry.GetPointsAttr().Get(&checker, motionFrames[0]);
    const int firstPointCount = checker.GetArraySize();
    bool isMeshTopologyHomogeneous = true;
    for (const float frame: motionFrames) {
        usdGeometry.GetPointsAttr().Get(&checker, frame);
        if (checker.GetArraySize() != firstPointCount) {
            isMeshTopologyHomogeneous = false;
            break;
        }
    }

    return isMeshTopologyHomogeneous;
}

template
bool
isTopologyHomogeneous<UsdGeomMesh>(const UsdGeomMesh&,
                                   const std::vector<float>&);

template
bool
isTopologyHomogeneous<UsdGeomBasisCurves>(const UsdGeomBasisCurves&,
                                          const std::vector<float>&);

template
bool
isTopologyHomogeneous<UsdGeomPoints>(const UsdGeomPoints&,
                                     const std::vector<float>&);
template<typename GeomType>
scene_rdl2::rdl2::MotionBlurType
getMotionBlurType(const GeomType& usdGeometry,
                  scene_rdl2::rdl2::MotionBlurType requestedMbType,
                  std::vector<float>& motionFrames,
                  float currentFrame)
{
    scene_rdl2::rdl2::MotionBlurType result = scene_rdl2::rdl2::MotionBlurType::STATIC;
    const int numMotionFrames = motionFrames.size();
    UsdGeomPrimvarsAPI primvarsApi(usdGeometry);
    switch (requestedMbType) {

        case scene_rdl2::rdl2::MotionBlurType::STATIC:
            break;

        case scene_rdl2::rdl2::MotionBlurType::VELOCITY:
            if (numMotionFrames >= 2  &&  usdGeometry.GetVelocitiesAttr().ValueMightBeTimeVarying()) {
                result = scene_rdl2::rdl2::MotionBlurType::VELOCITY;
            }
            break;

        case scene_rdl2::rdl2::MotionBlurType::FRAME_DELTA:
            if (numMotionFrames >= 2  &&  usdGeometry.GetPointsAttr().ValueMightBeTimeVarying()) {
                result = scene_rdl2::rdl2::MotionBlurType::FRAME_DELTA;
            }
            break;

        case scene_rdl2::rdl2::MotionBlurType::ACCELERATION:
            if (numMotionFrames >= 2  &&  usdGeometry.GetVelocitiesAttr().ValueMightBeTimeVarying()
                &&  primvarsApi.HasPrimvar(TfToken("accel"))) {
                result = scene_rdl2::rdl2::MotionBlurType::ACCELERATION;
            }
            break;

        case scene_rdl2::rdl2::MotionBlurType::HERMITE:
            if (numMotionFrames >= 2  &&  usdGeometry.GetPointsAttr().ValueMightBeTimeVarying() &&
                usdGeometry.GetVelocitiesAttr().ValueMightBeTimeVarying()) {
                result = scene_rdl2::rdl2::MotionBlurType::HERMITE;
            }
            break;

        case scene_rdl2::rdl2::MotionBlurType::BEST:
            if (numMotionFrames >= 2) {
                const bool isMeshTopologyHomogeneous = isTopologyHomogeneous<GeomType>(usdGeometry, motionFrames);

                if (isMeshTopologyHomogeneous &&  usdGeometry.GetVelocitiesAttr().ValueMightBeTimeVarying()) {
                    result = scene_rdl2::rdl2::MotionBlurType::HERMITE;
                } else if (isMeshTopologyHomogeneous && usdGeometry.GetPointsAttr().ValueMightBeTimeVarying()) {
                    result = scene_rdl2::rdl2::MotionBlurType::FRAME_DELTA;
                } else if (usdGeometry.GetVelocitiesAttr().ValueMightBeTimeVarying()) {
                    if (primvarsApi.HasPrimvar(TfToken("accel"))) {
                        result = scene_rdl2::rdl2::MotionBlurType::ACCELERATION;
                    } else {
                        result = scene_rdl2::rdl2::MotionBlurType::VELOCITY;
                    }
                }
            }
            break;

        default:
            break;
    }

    if (result == scene_rdl2::rdl2::MotionBlurType::STATIC ||
        result == scene_rdl2::rdl2::MotionBlurType::ACCELERATION ||
        result == scene_rdl2::rdl2::MotionBlurType::VELOCITY) {
        // We should only sample from the current frame in these cases
        motionFrames.clear();
        motionFrames.push_back(currentFrame);
    }

    return result;
}

template
scene_rdl2::rdl2::MotionBlurType
getMotionBlurType<UsdGeomMesh>(const UsdGeomMesh&,
                               scene_rdl2::rdl2::MotionBlurType,
                               std::vector<float>&,
                               float);

template
scene_rdl2::rdl2::MotionBlurType
getMotionBlurType<UsdGeomBasisCurves>(const UsdGeomBasisCurves&,
                                      scene_rdl2::rdl2::MotionBlurType,
                                      std::vector<float>&,
                                      float);

template
scene_rdl2::rdl2::MotionBlurType
getMotionBlurType<UsdGeomPoints>(const UsdGeomPoints&,
                                 scene_rdl2::rdl2::MotionBlurType,
                                 std::vector<float>&,
                                 float);

bool
primvarTypeIsCorrect(const UsdGeomPrimvar& primvar,
                     const AttributeKey& rdlAttrKey)
{
    bool result = false;

    const SdfValueTypeName usdType = primvar.GetTypeName().GetScalarType();

    switch (rdlAttrKey.getType()) {
    case scene_rdl2::rdl2::TYPE_BOOL:
        result = usdType == SdfValueTypeNames->Bool;
        break;
    case scene_rdl2::rdl2::TYPE_INT:
        result = usdType == SdfValueTypeNames->Int;
        break;
    case scene_rdl2::rdl2::TYPE_LONG:
        result = usdType == SdfValueTypeNames->Int64;
        break;
    case scene_rdl2::rdl2::TYPE_FLOAT:
        result = usdType == SdfValueTypeNames->Float;
        break;
    case scene_rdl2::rdl2::TYPE_STRING:
        result = usdType == SdfValueTypeNames->String;
        break;
    case scene_rdl2::rdl2::TYPE_RGB:
        result = usdType == SdfValueTypeNames->Color3f;
        break;
    case scene_rdl2::rdl2::TYPE_RGBA:
        result = usdType == SdfValueTypeNames->Color4f;
        break;
    case scene_rdl2::rdl2::TYPE_VEC2F:
        result = usdType == SdfValueTypeNames->Float2 || usdType == SdfValueTypeNames->TexCoord2f;
        break;
    case scene_rdl2::rdl2::TYPE_VEC3F:
        result = usdType == SdfValueTypeNames->Float3 || usdType == SdfValueTypeNames->Vector3f ||
                 usdType == SdfValueTypeNames->Normal3f || usdType == SdfValueTypeNames->Point3f;
        break;
    case scene_rdl2::rdl2::TYPE_MAT4F:
        result = usdType == SdfValueTypeNames->Matrix4d;
        break;
    default:
        break;
    }
    return result;
}


template<typename GeomType>
bool
isLeftHanded(const GeomType& usdGeometry)
{
    TfToken orientation;
    usdGeometry.GetOrientationAttr().Get(&orientation,
                                  UsdTimeCode::EarliestTime());
    return orientation == UsdGeomTokens->leftHanded;
}

template
bool
isLeftHanded<UsdGeomMesh>(const UsdGeomMesh&);

template
bool
isLeftHanded<UsdGeomBasisCurves>(const UsdGeomBasisCurves&);

template
bool
isLeftHanded<UsdGeomPoints>(const UsdGeomPoints&);

template<>
void
getPrimVertexCounts<UsdGeomMesh>(const UsdGeomMesh& mesh,
                                 VtArray<int>& primVertexCounts,
                                 float frame)
{
    mesh.GetFaceVertexCountsAttr().Get(&primVertexCounts, frame);
}

template<>
void
getPrimVertexCounts<UsdGeomBasisCurves>(const UsdGeomBasisCurves& curves,
                                        VtArray<int>& primVertexCounts,
                                        float frame)
{
    curves.GetCurveVertexCountsAttr().Get(&primVertexCounts, frame);
}

template<>
void
getPrimVertexCounts<UsdGeomPoints>(const UsdGeomPoints& points,
                                   VtArray<int>& primVertexCounts,
                                   float frame)
{
    VtArray <GfVec3f> points0;
    points.GetPointsAttr().Get(&points0, frame);
    primVertexCounts.resize(points0.size());
    std::fill(std::begin(primVertexCounts), std::end(primVertexCounts), 1);
}

template<typename GeomType>
void
addPrimvar(const scene_rdl2::rdl2::Geometry *rdlGeometry,
           const GeomType& usdGeometry,
           const AttributeKey& rdlAttrKey,
           PrimitiveAttributeTable& table,
           const std::vector<float>& motionFrames)
{
    UsdGeomPrimvar primvar;
    if (!getPrimvar<GeomType>(rdlGeometry, usdGeometry, rdlAttrKey, primvar, motionFrames)) {
        return;
    }

    AttributeRate attrRate;
    if (!getPrimvarRate(rdlGeometry, primvar, rdlAttrKey, attrRate)) {
        return;
    }

    const float frame = motionFrames.back();

    // For types that cannot be interpolated (i.e. bool, int, etc.) we call
    // setStaticPrimvar which only accepts constant and uniform rates.  For
    // types that we can interpolate we call setPrimvar which accepts
    // constant, uniform, vertex, varying, and face-varying rates.
    switch (rdlAttrKey.getType()) {
    case scene_rdl2::rdl2::TYPE_BOOL:
        setStaticPrimvar<bool>(primvar, table, frame, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_INT:
        setStaticPrimvar<int>(primvar, table, frame, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_LONG:
        setStaticPrimvar<int64_t>(primvar, table, frame, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_FLOAT:
        setPrimvar<float, float, GeomType>(rdlGeometry, usdGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_STRING:
        setStaticPrimvar<std::string>(primvar, table, frame, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_RGB:
        setPrimvar<scene_rdl2::math::Color, GfVec3f, GeomType>(rdlGeometry, usdGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_RGBA:
        setPrimvar<scene_rdl2::math::Color4, GfVec4f, GeomType>(rdlGeometry, usdGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_VEC2F:
        setPrimvar<scene_rdl2::math::Vec2f, GfVec2f, GeomType>(rdlGeometry, usdGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_VEC3F:
        setPrimvar<scene_rdl2::math::Vec3f, GfVec3f, GeomType>(rdlGeometry, usdGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_MAT4F:
        setPrimvar<scene_rdl2::math::Mat4f, GfMatrix4d, GeomType>(rdlGeometry, usdGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    default:
        break;
    }
}

void
addPointInstancerPrimvar(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                         const UsdGeomPointInstancer& pointInstancer,
                         const AttributeKey& rdlAttrKey,
                         PrimitiveAttributeTable& table,
                         const std::vector<float>& motionFrames)
{
    UsdGeomPrimvar primvar;
    if (!getPrimvar<UsdGeomPointInstancer>(rdlGeometry, pointInstancer, rdlAttrKey, primvar, motionFrames)) {
        return;
    }

    AttributeRate attrRate;
    if (!getPrimvarRate(rdlGeometry, primvar, rdlAttrKey, attrRate)) {
        return;
    }

    const float frame = motionFrames.back();

    // For types that cannot be interpolated (i.e. bool, int, etc.) we call
    // setStaticPrimvar which only accepts constant and uniform rates.  For
    // types that we can interpolate we call setPointInstancerPrimvar which
    // accepts constant, vertex, and varying rates.
    switch (rdlAttrKey.getType()) {
    case scene_rdl2::rdl2::TYPE_BOOL:
        setStaticPrimvar<bool>(primvar, table, frame, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_INT:
        setStaticPrimvar<int>(primvar, table, frame, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_LONG:
        setStaticPrimvar<int64_t>(primvar, table, frame, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_FLOAT:
        setPointInstancerPrimvar<float, float>(rdlGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_STRING:
        setStaticPrimvar<std::string>(primvar, table, frame, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_RGB:
        setPointInstancerPrimvar<scene_rdl2::math::Color, GfVec3f>(rdlGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_RGBA:
        setPointInstancerPrimvar<scene_rdl2::math::Color4, GfVec4f>(rdlGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_VEC2F:
        setPointInstancerPrimvar<scene_rdl2::math::Vec2f, GfVec2f>(rdlGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_VEC3F:
        setPointInstancerPrimvar<scene_rdl2::math::Vec3f, GfVec3f>(rdlGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    case scene_rdl2::rdl2::TYPE_MAT4F:
        setPointInstancerPrimvar<scene_rdl2::math::Mat4f, GfMatrix4d>(rdlGeometry, primvar, table, motionFrames, rdlAttrKey, attrRate);
        break;
    default:
        break;
    }
}

void
addPointInstancerUVs(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                     const UsdGeomPointInstancer& pointInstancer,
                     PrimitiveAttributeTable& table,
                     const std::vector<float>& motionFrames)
{
    UsdGeomPrimvar primvar;
    if (!getPrimvar<UsdGeomPointInstancer>(rdlGeometry,
                                           pointInstancer,
                                           StandardAttributes::sUv,
                                           primvar,
                                           motionFrames)) {
        return;
    }

    AttributeRate attrRate;
    if (!getPrimvarRate(rdlGeometry,
                        primvar,
                        StandardAttributes::sUv,
                        attrRate)) {
        return;
    }

    std::vector<scene_rdl2::math::Vec2f> textureUV;
    getPointInstancerPrimvarVectorData<scene_rdl2::math::Vec2f, GfVec2f>(rdlGeometry,
                                                                         primvar,
                                                                         StandardAttributes::sUv,
                                                                         motionFrames,
                                                                         attrRate,
                                                                         textureUV);

    table.addAttribute(StandardAttributes::sUv,
                       attrRate,
                       std::move(textureUV));
}

void
addPointInstancerNormals(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                         const UsdGeomPointInstancer& pointInstancer,
                         PrimitiveAttributeTable& table,
                         const std::vector<float>& motionFrames)
{
    // Check for alternate "N" naming
    TypedAttributeKey<scene_rdl2::math::Vec3f> normalKey = StandardAttributes::sNormal;
    const UsdGeomPrimvarsAPI pvAPI(pointInstancer.GetPrim());
    if (!pvAPI.HasPrimvar(TfToken(StandardAttributes::sNormal.getName()))) {
        if (pvAPI.HasPrimvar(TfToken("N"))) {
            normalKey = TypedAttributeKey<scene_rdl2::math::Vec3f>("N");
        } else {
            return;
        }
    }

    UsdGeomPrimvar primvar;
    if (!getPrimvar<UsdGeomPointInstancer>(rdlGeometry,
                                           pointInstancer,
                                           normalKey,
                                           primvar,
                                           motionFrames)) {
        return;
    }

    AttributeRate attrRate;
    if (!getPrimvarRate(rdlGeometry,
                        primvar,
                        normalKey,
                        attrRate)) {
        return;
    }

    std::vector<scene_rdl2::math::Vec3f> normals;
    getPointInstancerPrimvarVectorData<scene_rdl2::math::Vec3f, GfVec3f>(rdlGeometry,
                                                             primvar,
                                                             normalKey,
                                                             motionFrames,
                                                             attrRate,
                                                             normals);

    // Validate normals
    for (size_t i = 0; i < normals.size(); ++i) {
        if (std::isnan(normals[i].x) ||
            std::isnan(normals[i].y) ||
            std::isnan(normals[i].z) ||
            !std::isfinite(normals[i].x) ||
            !std::isfinite(normals[i].y) ||
            !std::isfinite(normals[i].z) ||
            scene_rdl2::math::isZero(normals[i])) {
                throw std::runtime_error("Invalid normal normals on points");
        }

        normals[i] = scene_rdl2::math::normalize(normals[i]);
    }

    table.addAttribute(StandardAttributes::sNormal,
                       attrRate,
                       std::move(normals));
}

void
addPointInstancerDerivatives(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                             const UsdGeomPointInstancer& pointInstancer,
                             PrimitiveAttributeTable& table,
                             const std::vector<float>& motionFrames)
{
    UsdGeomPrimvar primvar;
    AttributeRate attrRate;

    // dPds
    if (!getPrimvar<UsdGeomPointInstancer>(rdlGeometry,
                                           pointInstancer,
                                           StandardAttributes::sdPds,
                                           primvar,
                                           motionFrames)) {
        return;
    }

    if (!getPrimvarRate(rdlGeometry,
                        primvar,
                        StandardAttributes::sdPds,
                        attrRate)) {
        return;
    }

    std::vector<scene_rdl2::math::Vec3f> dPds;
    getPointInstancerPrimvarVectorData<scene_rdl2::math::Vec3f, GfVec3f>(rdlGeometry,
                                                             primvar,
                                                             StandardAttributes::sdPds,
                                                             motionFrames,
                                                             attrRate,
                                                             dPds);

    // Validate dPds
    for (size_t i = 0; i < dPds.size(); ++i) {
        if (std::isnan(dPds[i].x) ||
            std::isnan(dPds[i].y) ||
            std::isnan(dPds[i].z) ||
            !std::isfinite(dPds[i].x) ||
            !std::isfinite(dPds[i].y) ||
            !std::isfinite(dPds[i].z) ||
            scene_rdl2::math::isZero(dPds[i])) {
                throw std::runtime_error("Invalid dPds on points");
        }
    }

    table.addAttribute(StandardAttributes::sdPds,
                       attrRate,
                       std::move(dPds));

    // dPdt
    if (!getPrimvar<UsdGeomPointInstancer>(rdlGeometry,
                                           pointInstancer,
                                           StandardAttributes::sdPdt,
                                           primvar,
                                           motionFrames)) {
        return;
    }

    if (!getPrimvarRate(rdlGeometry,
                        primvar,
                        StandardAttributes::sdPdt,
                        attrRate)) {
        return;
    }

    std::vector<scene_rdl2::math::Vec3f> dPdt;
    getPointInstancerPrimvarVectorData<scene_rdl2::math::Vec3f, GfVec3f>(rdlGeometry,
                                                             primvar,
                                                             StandardAttributes::sdPdt,
                                                             motionFrames,
                                                             attrRate,
                                                             dPdt);

    // Validate dPdt
    for (size_t i = 0; i < dPdt.size(); ++i) {
        if (std::isnan(dPdt[i].x) ||
            std::isnan(dPdt[i].y) ||
            std::isnan(dPdt[i].z) ||
            !std::isfinite(dPdt[i].x) ||
            !std::isfinite(dPdt[i].y) ||
            !std::isfinite(dPdt[i].z) ||
            scene_rdl2::math::isZero(dPdt[i])) {
                throw std::runtime_error("Invalid dPdt on points");
        }
    }

    table.addAttribute(StandardAttributes::sdPdt,
                       attrRate,
                       std::move(dPdt));
}

template
void
addPrimvar<UsdGeomMesh>(const scene_rdl2::rdl2::Geometry*,
                        const UsdGeomMesh&,
                        const AttributeKey&,
                        PrimitiveAttributeTable&,
                        const std::vector<float>&);

template
void
addPrimvar<UsdGeomBasisCurves>(const scene_rdl2::rdl2::Geometry*,
                               const UsdGeomBasisCurves&,
                               const AttributeKey&,
                               PrimitiveAttributeTable&,
                               const std::vector<float>&);

template
void
addPrimvar<UsdGeomPoints>(const scene_rdl2::rdl2::Geometry*,
                          const UsdGeomPoints&,
                          const AttributeKey&,
                          PrimitiveAttributeTable&,
                          const std::vector<float>&);

template<typename GeomType>
VertexBuffer<Vec3fa, InterleavedTraits>
getVertices(const scene_rdl2::rdl2::Geometry *rdlGeometry,
            const GeomType& usdGeometry,
            PrimitiveAttributeTable& table,
            scene_rdl2::rdl2::MotionBlurType mbType,
            const std::vector<float>& motionFrames)
{
    size_t samplesCount = motionFrames.size();

    VtArray <GfVec3f> points0, points1, velocity0, velocity1;

    UsdSkelCache skelCache;

    if (UsdSkelRoot skelRoot = UsdSkelRoot::Find(usdGeometry.GetPrim())) {
        
        // Make sure skeletal animation is included in the stage mask
        // by expanding the mask to include any UsdSkel relationships
        // if there is a mask present.
        UsdStageRefPtr stage = usdGeometry.GetPrim().GetStage();
        if(!stage->GetPopulationMask().IncludesSubtree(SdfPath::AbsoluteRootPath()))
            stage->ExpandPopulationMask(expandSkelMask);

#if PXR_VERSION >= 2011
        skelCache.Populate(skelRoot, pxr::UsdPrimDefaultPredicate);
#else
        skelCache.Populate(skelRoot);
#endif

        if (UsdSkelSkinningQuery skinningQuery = skelCache.GetSkinningQuery(usdGeometry.GetPrim())) {
            VtMatrix4dArray xforms0, xforms1;
            UsdSkelSkeleton skel = UsdSkelBindingAPI(usdGeometry.GetPrim()).GetInheritedSkeleton();
            UsdSkelSkeletonQuery skelQuery = skelCache.GetSkelQuery(skel);
            skelQuery.ComputeSkinningTransforms(&xforms0, motionFrames[0]);
            skelQuery.ComputeSkinningTransforms(&xforms1, motionFrames[1]);
            usdGeometry.GetPointsAttr().Get(&points0, motionFrames[0]);
            usdGeometry.GetPointsAttr().Get(&points1, motionFrames[1]);
            skinningQuery.ComputeSkinnedPoints(xforms0, &points0, motionFrames[0]);
            skinningQuery.ComputeSkinnedPoints(xforms1, &points1, motionFrames[1]);
        }
    } else {
        usdGeometry.GetPointsAttr().Get(&points0, motionFrames[0]);
        usdGeometry.GetVelocitiesAttr().Get(&velocity0, motionFrames[0]);

        if (samplesCount > 1) {
            // FIXME once Moonray adds multiple motion samples
            usdGeometry.GetPointsAttr().Get(&points1, motionFrames[1]);
            usdGeometry.GetVelocitiesAttr().Get(&velocity1, motionFrames[1]);
        }
    }

    size_t vertexCount = points0.size();

    UsdGeomPrimvarsAPI primvarsApi(usdGeometry);

    switch (mbType) {

        case scene_rdl2::rdl2::MotionBlurType::STATIC:
        {
            VertexBuffer<Vec3fa, InterleavedTraits> vertices(vertexCount, 1);
            for (size_t i = 0; i < vertexCount; ++i) {
                vertices(i) = Vec3fa(points0[i][0], points0[i][1], points0[i][2], 0.f);
            }
            return vertices;
        }

        case scene_rdl2::rdl2::MotionBlurType::VELOCITY:
        {
            // If data sufficient, extract positions and velocities
            if (velocity0.size() == vertexCount) {
                VertexBuffer<Vec3fa, InterleavedTraits> vertices(vertexCount, 1);
                std::vector<Vec3f> velocities(vertexCount);
                for (size_t i = 0; i < vertexCount; ++i) {
                    vertices(i)   = Vec3fa(points0[i][0], points0[i][1], points0[i][2], 0.f);
                    velocities[i] = Vec3f(velocity0[i][0], velocity0[i][1], velocity0[i][2]);
                }
                table.addAttribute(StandardAttributes::sVelocity, RATE_VERTEX, std::move(velocities));
                return vertices;
            }
            break;
        }

        case scene_rdl2::rdl2::MotionBlurType::ACCELERATION:
        {
            // Get entry count of acceleration array, if present
            int accelCount = 0;
            if (primvarsApi.HasPrimvar(TfToken("accel"))) {
                VtArray<GfVec3f> accelValues;
                const UsdGeomPrimvar primvar = primvarsApi.GetPrimvar(TfToken("accel"));
                primvar.Get(&accelValues, motionFrames[0]);
                accelCount = accelValues.size();
            }

            // If data sufficient, extract positions, velocities and accelerations
            if (velocity0.size() == vertexCount  &&  accelCount == vertexCount) {
                VertexBuffer<Vec3fa, InterleavedTraits> vertices(vertexCount, 1);
                std::vector<Vec3f> velocities(vertexCount);
                for (size_t i = 0; i < vertexCount; ++i) {
                    vertices(i)   = Vec3fa(points0[i][0], points0[i][1], points0[i][2], 0.f);
                    velocities[i] = Vec3f(velocity0[i][0], velocity0[i][1], velocity0[i][2]);
                }
                table.addAttribute(StandardAttributes::sVelocity, RATE_VERTEX, std::move(velocities));
                addPrimvar(rdlGeometry, usdGeometry, TypedAttributeKey<Vec3f>("accel"), table, motionFrames);
                return vertices;
            }
            break;
        }

        case scene_rdl2::rdl2::MotionBlurType::FRAME_DELTA:
        {
            // If data sufficient, extract positions from consecutive frames
            if ((samplesCount > 1) && (vertexCount == points1.size())) {
                VertexBuffer<Vec3fa, InterleavedTraits> vertices(vertexCount, 2);
                for (size_t i = 0; i < vertexCount; ++i) {
                    vertices(i, 0) = Vec3fa(points0[i][0], points0[i][1], points0[i][2], 0.f);
                    vertices(i, 1) = Vec3fa(points1[i][0], points1[i][1], points1[i][2], 0.f);
                }
                return vertices;
            }
            break;
        }

        case scene_rdl2::rdl2::MotionBlurType::HERMITE:
        {
            // If data sufficient, extract positions and velocities from consecutive frames
            if (samplesCount > 1  &&  vertexCount == points1.size()  &&
                velocity0.size() == vertexCount  &&  velocity1.size() == vertexCount) {
                VertexBuffer<Vec3fa, InterleavedTraits> vertices(vertexCount, 2);
                std::vector<std::vector<Vec3f>> velocities(2, std::vector<Vec3f>(vertexCount));
                for (size_t i = 0; i < vertexCount; ++i) {
                    vertices(i, 0) = Vec3fa(points0[i][0], points0[i][1], points0[i][2], 0.f);
                    vertices(i, 1) = Vec3fa(points1[i][0], points1[i][1], points1[i][2], 0.f);
                    velocities[0][i] = Vec3f(velocity0[i][0], velocity0[i][1], velocity0[i][2]);
                    velocities[1][i] = Vec3f(velocity1[i][0], velocity1[i][1], velocity1[i][2]);
                }
                table.addAttribute(StandardAttributes::sVelocity, RATE_VERTEX, std::move(velocities));
                return vertices;
            }
            break;
        }

        default:
        {
            break;
        }
    } // end of switch()

    rdlGeometry->warn("Unknown motion blur type.  Falling back to static.");
    VertexBuffer<Vec3fa, InterleavedTraits> vertices(vertexCount, 1);
    for (size_t i = 0; i < vertexCount; ++i) {
        vertices(i) = Vec3fa(points0[i][0], points0[i][1], points0[i][2], 0.f);
    }
    return vertices;
}

template
VertexBuffer<Vec3fa, InterleavedTraits>
getVertices<UsdGeomMesh>(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                         const UsdGeomMesh& usdGeometry,
                         PrimitiveAttributeTable& table,
                         scene_rdl2::rdl2::MotionBlurType mbType,
                         const std::vector<float>& motionFrames);

template
VertexBuffer<Vec3fa, InterleavedTraits>
getVertices<UsdGeomBasisCurves>(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                                const UsdGeomBasisCurves& usdGeometry,
                                PrimitiveAttributeTable& table,
                                scene_rdl2::rdl2::MotionBlurType mbType,
                                const std::vector<float>& motionFrames);

template
VertexBuffer<Vec3fa, InterleavedTraits>
getVertices<UsdGeomPoints>(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                           const UsdGeomPoints& usdGeometry,
                           PrimitiveAttributeTable& table,
                           scene_rdl2::rdl2::MotionBlurType mbType,
                           const std::vector<float>& motionFrames);

} // namespace usd
} // namespace moonshine

