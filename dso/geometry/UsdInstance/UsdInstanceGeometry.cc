// Copyright 2023-2024 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//  UsdInstanceGeometry is a procedural that can generate one or more Moonray
//  primitives from a specified geometry Prim on a USD stage.


#include "attributes.cc"

#include <moonshine_usd/geometry/usd/Usd.h>

#include <moonray/rendering/shading/AttributeKey.h>
#include <moonray/rendering/geom/Api.h>
#include <moonray/rendering/geom/ProceduralLeaf.h>
#include <scene_rdl2/scene/rdl2/rdl2.h>
#include <scene_rdl2/common/platform/Platform.h>

#include <pxr/pxr.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usd/stageCacheContext.h>
#include <pxr/usd/usdGeom/pointInstancer.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdSkel/root.h>

using namespace moonray;
using namespace moonray::geom;
using namespace moonray::shading;

static const TypedAttributeKey<scene_rdl2::math::Mat4f> instanceLevelKeys[] = {
    moonray::shading::StandardAttributes::sInstanceTransformLevel0,
    moonray::shading::StandardAttributes::sInstanceTransformLevel1,
    moonray::shading::StandardAttributes::sInstanceTransformLevel2,
    moonray::shading::StandardAttributes::sInstanceTransformLevel3,
    moonray::shading::StandardAttributes::sInstanceTransformLevel4
};

RDL2_DSO_CLASS_BEGIN(UsdInstanceGeometry, scene_rdl2::rdl2::Geometry)

public:
    RDL2_DSO_DEFAULT_CTOR(UsdInstanceGeometry)
    moonray::geom::Procedural* createProcedural() const override;
    void destroyProcedural() const override;
    bool deformed() const override;
    void resetDeformed() override;

RDL2_DSO_CLASS_END(UsdInstanceGeometry)

namespace {

const scene_rdl2::rdl2::String sDefaultPartName("");

scene_rdl2::math::Xform3f
usdMatrixToXform(const GfMatrix4d& m) {
    const double* d = m.data();
    return scene_rdl2::math::Xform3f(d[0],  d[1],  d[2],
                                     d[4],  d[5],  d[6],
                                     d[8],  d[9],  d[10],
                                     d[12], d[13], d[14]);
}

bool
isValidXform(const XformSamples& usdXforms)
{
    for (const auto& xform : usdXforms) {
        if (!scene_rdl2::math::isFinite(xform.row0()) ||
            !scene_rdl2::math::isFinite(xform.row1()) ||
            !scene_rdl2::math::isFinite(xform.row2()) ||
            !scene_rdl2::math::isFinite(xform.row3())) {
            return false;
        }
    }
    return true;
}

// Get the motion blur frames from the geometry context
void
getMotionFrames(const GenerateContext& generateContext,
                const float currentFrame,
                std::vector<float>& motionFrames,
                std::vector<UsdTimeCode>& times)
{
    if (generateContext.isMotionBlurOn()) { 
        for (const auto motionStep : generateContext.getMotionSteps()) {
                const float sampleNumber = scene_rdl2::rdl2::computeSampleNumber(currentFrame,
                                                                     motionStep);
            motionFrames.push_back(sampleNumber);
            times.push_back(UsdTimeCode(sampleNumber));
        }
    } else {
        motionFrames.push_back(currentFrame);
        times.push_back(UsdTimeCode(currentFrame));
    }
}

void
transferPrimitiveAttributes(const PrimitiveAttributeTable& pointsPAT,
                            PrimitiveAttributeTable& instancePAT,
                            const int instanceIndex)
{
        for (const auto& kv : pointsPAT) {
            const AttributeKey key = kv.first;

            if (!kv.second[0]) { continue; } // skip this invalid attribute
            const PrimitiveAttributeBase& attr = *kv.second[0];

            size_t index = 0;
            if (attr.getRate() != RATE_CONSTANT &&
                attr.getRate() != RATE_UNIFORM) {
                index = instanceIndex;
            }

            switch (key.getType()) {
            case scene_rdl2::rdl2::TYPE_BOOL:
                instancePAT.addAttribute(TypedAttributeKey<bool>(key),
                                         RATE_CONSTANT,
                                         std::vector<bool>({attr.as<bool>()[index]}));
                break;
            case scene_rdl2::rdl2::TYPE_INT:
                instancePAT.addAttribute(TypedAttributeKey<int>(key),
                                         RATE_CONSTANT,
                                         std::vector<int>({attr.as<int>()[index]}));
                break;
            case scene_rdl2::rdl2::TYPE_LONG:
                instancePAT.addAttribute(TypedAttributeKey<long>(key),
                                         RATE_CONSTANT,
                                         std::vector<long>({attr.as<long>()[index]}));
                break;
            case scene_rdl2::rdl2::TYPE_FLOAT:
                instancePAT.addAttribute(TypedAttributeKey<float>(key),
                                         RATE_CONSTANT,
                                         std::vector<float>({attr.as<float>()[index]}));
                break;
            case scene_rdl2::rdl2::TYPE_STRING:
                instancePAT.addAttribute(TypedAttributeKey<std::string>(key),
                                         RATE_CONSTANT,
                                         {attr.as<std::string>()[index]});
                break;
            case scene_rdl2::rdl2::TYPE_RGB:
                instancePAT.addAttribute(TypedAttributeKey<scene_rdl2::math::Color>(key),
                                         RATE_CONSTANT,
                                         {attr.as<scene_rdl2::math::Color>()[index]});
                break;
            case scene_rdl2::rdl2::TYPE_RGBA:
                instancePAT.addAttribute(TypedAttributeKey<scene_rdl2::math::Color4>(key),
                                         RATE_CONSTANT,
                                         {attr.as<scene_rdl2::math::Color4>()[index]});
                break;
            case scene_rdl2::rdl2::TYPE_VEC2F:
                instancePAT.addAttribute(TypedAttributeKey<Vec2f>(key),
                                         RATE_CONSTANT,
                                         {attr.as<Vec2f>()[index]});
                break;
            case scene_rdl2::rdl2::TYPE_VEC3F:
                instancePAT.addAttribute(TypedAttributeKey<Vec3f>(key),
                                 RATE_CONSTANT,
                                 {attr.as<Vec3f>()[index]});
                break;
            case scene_rdl2::rdl2::TYPE_MAT4F:
                instancePAT.addAttribute(TypedAttributeKey<scene_rdl2::math::Mat4f>(key),
                                         RATE_CONSTANT,
                                         {attr.as<scene_rdl2::math::Mat4f>()[index]});
                break;
            default:
                break;
            }
        }
}

}  // anonymous namespace

namespace moonray {
namespace geom {

PXR_NAMESPACE_USING_DIRECTIVE

class UsdInstanceGeometryProcedural : public ProceduralLeaf
{
public:
    UsdInstanceGeometryProcedural(const State &state)
        : ProceduralLeaf(state)
    {}

    void generate(const GenerateContext &generateContext,
            const XformSamples &parent2render) override;

    void update(const UpdateContext &updateContext,
            const XformSamples &parent2render) override;
};

void
UsdInstanceGeometryProcedural::generate(const GenerateContext& generateContext,
                                        const XformSamples& parent2render)
{
    const scene_rdl2::rdl2::Geometry *rdlGeometry = generateContext.getRdlGeometry();
    const UsdInstanceGeometry *usdInstanceGeometry =
        static_cast<const UsdInstanceGeometry*>(rdlGeometry);

    const scene_rdl2::math::Mat4f nodeXform = scene_rdl2::math::toFloat(rdlGeometry->get(scene_rdl2::rdl2::Node::sNodeXformKey,
                                                                 scene_rdl2::rdl2::TIMESTEP_BEGIN));

    const auto& reference = rdlGeometry->get(scene_rdl2::rdl2::Geometry::sReferenceGeometries);
    if (reference.empty()) {
        rdlGeometry->error("Did not find any reference geometry. "
                           "Please make sure the \"references\" field contains "
                           "at least one source reference geometry");
        return;
    }

    const size_t numReferences = reference.size();

    // Load the stage and primitive
    const std::string& stageFile = usdInstanceGeometry->get(attrStage);
    if (stageFile.empty()) {
        rdlGeometry->error("\"stage\" parameter is empty");
        return;
    }

    const std::string& pointInstancerPath = usdInstanceGeometry->get(attrPointInstancerPath);
    if (pointInstancerPath.empty()) {
        rdlGeometry->error("\"point_instancer_path\" parameter is empty");
        return;
    }

    // Create shared primitives for the referenced primitives
    std::vector<std::shared_ptr<SharedPrimitive>> refPrimitiveGroup(numReferences, nullptr);
    std::vector<moonray::shading::XformSamples> refXforms(numReferences);
    std::vector<float> refShadowRayEpsilons;
    for (size_t i = 0; i < numReferences; ++i) {
        if (reference[i]->isA<scene_rdl2::rdl2::Geometry>()) {
            const scene_rdl2::rdl2::Geometry* g = reference[i]->asA<scene_rdl2::rdl2::Geometry>();
            refShadowRayEpsilons.push_back(g->getShadowRayEpsilon());
            refPrimitiveGroup[i] = g->getProcedural()->getReference();

            scene_rdl2::math::Mat4f l2r0 = scene_rdl2::math::toFloat(g->get(scene_rdl2::rdl2::Node::sNodeXformKey, scene_rdl2::rdl2::TIMESTEP_BEGIN));
            refXforms[i].push_back(scene_rdl2::math::Xform3f(l2r0[0][0], l2r0[0][1], l2r0[0][2],
                                                 l2r0[1][0], l2r0[1][1], l2r0[1][2],
                                                 l2r0[2][0], l2r0[2][1], l2r0[2][2],
                                                 l2r0[3][0], l2r0[3][1], l2r0[3][2]));

            scene_rdl2::math::Mat4f l2r1 = scene_rdl2::math::toFloat(g->get(scene_rdl2::rdl2::Node::sNodeXformKey, scene_rdl2::rdl2::TIMESTEP_END));
            refXforms[i].push_back(scene_rdl2::math::Xform3f(l2r1[0][0], l2r1[0][1], l2r1[0][2],
                                                 l2r1[1][0], l2r1[1][1], l2r1[1][2],
                                                 l2r1[2][0], l2r1[2][1], l2r1[2][2],
                                                     l2r1[3][0], l2r1[3][1], l2r1[3][2]));
        }
    }

    UsdStageRefPtr stage;
    if (usdInstanceGeometry->get(attrUseStageCache)) {
        stage = UsdStage::Open(stageFile);
    } else {
        UsdStagePopulationMask mask;
        mask.Add(SdfPath(pointInstancerPath));
        stage = UsdStage::OpenMasked(stageFile, mask);
    }

    if (!stage) {
        rdlGeometry->error("Could not open stage: ", stageFile);
        return;
    }

    const UsdPrim prim = stage->GetPrimAtPath(SdfPath(pointInstancerPath));
    if (!prim) {
        rdlGeometry->error("Could not find prim point instancer path: ", pointInstancerPath);
        return;
    }

    const UsdGeomPointInstancer pointInstancer(prim);
    if (!pointInstancer) {
        rdlGeometry->error("Path does not point to point instancer type: ", pointInstancerPath);
        return;
    }

    // Get the two frames to evaluate based on motion blur settings
    const float currentFrame = usdInstanceGeometry->get(attrUseEvaluationFrame) ?
                               usdInstanceGeometry->get(attrEvaluationFrame) :
                               generateContext.getCurrentFrame();
    std::vector<float> motionFrames;
    std::vector<UsdTimeCode> times;
    getMotionFrames(generateContext, currentFrame, motionFrames, times);

    // Get an ID mask to use against any returned values
    std::vector<bool> idMask = pointInstancer.ComputeMaskAtTime(currentFrame);

    // Get the indices of the referenced objects to instance
    VtArray<int> indices;
    pointInstancer.GetProtoIndicesAttr().Get(&indices, currentFrame);
    const size_t numIndices = indices.size();
    if (numIndices == 0) {
        rdlGeometry->error("Point instancer has no protoIndices attribute");
        return;
    }
    for (size_t i = 0; i < numIndices; ++i) {
        if (indices[i] > static_cast<int>(numReferences)) {
            rdlGeometry->error("Point instancer protoIndices index: ",
                               indices[i],
                               " is greater then the number of references: ",
                               numReferences);
            return;
        }
    }

    // Get the instance transforms for the two motion frames
    std::vector<VtArray<GfMatrix4d>> usdXforms;
    if (!pointInstancer.ComputeInstanceTransformsAtTimes(
        &usdXforms,
        times,
        UsdTimeCode(currentFrame),
        usdInstanceGeometry->get(attrUseReferenceXforms) ?
        pointInstancer.IncludeProtoXform :
        pointInstancer.ExcludeProtoXform,
        pointInstancer.IgnoreMask)) {

        // If ComputeInstanceTransformsAtTimes fails, it might be due to a
        // mismatch between the size of the "protoindices" attribute and
        // the other instancer attributes.   Try to fix this by resizing
        // any instance attribute that doesn't match the size of "protoindices".
        VtArray <GfVec3f> positions;
        pointInstancer.GetPositionsAttr().Get(&positions, currentFrame);
        if (positions.empty()) {
            rdlGeometry->error("Point instancer has no positions attribute");
            return;
        }
        if (!positions.empty() && positions.size() != numIndices) {
            positions.resize(numIndices);
            pointInstancer.GetPositionsAttr().Set(positions, currentFrame);
        }

        VtArray <GfQuath> orientations;
        pointInstancer.GetOrientationsAttr().Get(&orientations, currentFrame);
        if (!orientations.empty() && orientations.size() != numIndices) {
            orientations.resize(numIndices);
            pointInstancer.GetOrientationsAttr().Set(orientations, currentFrame);
        }

        VtArray <GfVec3f> scales;
        pointInstancer.GetScalesAttr().Get(&scales, currentFrame);
        if (!scales.empty() && scales.size() != numIndices) {
            scales.resize(numIndices);
            pointInstancer.GetScalesAttr().Set(scales, currentFrame);
        }

        VtArray <GfVec3f> velocities;
        pointInstancer.GetVelocitiesAttr().Get(&velocities, currentFrame);
        if (!velocities.empty() && velocities.size() != numIndices) {
            velocities.resize(numIndices);
            pointInstancer.GetVelocitiesAttr().Set(velocities, currentFrame);
        }

        VtArray <GfVec3f> accelerations;
        pointInstancer.GetAccelerationsAttr().Get(&accelerations, currentFrame);
        if (!accelerations.empty() && accelerations.size() != numIndices) {
            accelerations.resize(numIndices);
            pointInstancer.GetAccelerationsAttr().Set(accelerations, currentFrame);
        }

        VtArray <GfVec3f> angularVelocities;
        pointInstancer.GetAngularVelocitiesAttr().Get(&angularVelocities, currentFrame);
        if (!angularVelocities.empty() && angularVelocities.size() != numIndices) {
            angularVelocities.resize(numIndices);
            pointInstancer.GetAngularVelocitiesAttr().Set(angularVelocities, currentFrame);
        }

        VtArray <int64_t> invisibleIds;
        pointInstancer.GetInvisibleIdsAttr().Get(&invisibleIds, currentFrame);
        if (!invisibleIds.empty() && invisibleIds.size() != numIndices) {
            invisibleIds.resize(numIndices);
            pointInstancer.GetInvisibleIdsAttr().Set(invisibleIds, currentFrame);
        }

        // Try again
        if (!pointInstancer.ComputeInstanceTransformsAtTimes(
            &usdXforms,
            times,
            UsdTimeCode(currentFrame),
            usdInstanceGeometry->get(attrUseReferenceXforms) ?
            pointInstancer.IncludeProtoXform :
            pointInstancer.ExcludeProtoXform,
            pointInstancer.IgnoreMask)) {
            
            // Error out if it still fails
            rdlGeometry->error("Problem computing instance transforms");
            return;
        } else {
            rdlGeometry->warn("Mismatch between size of protoIndices and one or ",
                              "more instancer attributes.  Resized instancer ",
                              "attributes.");
        }
    }

    const size_t usdXformsCount = usdXforms[0].size();

    // Support the "instance_level" attribute
    const int instanceLevel = usdInstanceGeometry->get(attrInstanceLevel);
    bool addInstanceTransformAttribute = false;
    TypedAttributeKey<scene_rdl2::math::Mat4f> instanceLevelKey = instanceLevelKeys[instanceLevel];

    bool addInstanceObjectTransformAttribute = false;

    PrimitiveAttributeTable pointsPAT;

    // optional texture uv
    moonshine::usd::addPointInstancerUVs(rdlGeometry,
                                         pointInstancer,
                                         pointsPAT,
                                         motionFrames);

    try {
        // optional normals
        moonshine::usd::addPointInstancerNormals(rdlGeometry,
                                                 pointInstancer,
                                                 pointsPAT,
                                                 motionFrames);

        // optional dPds dPdt
        moonshine::usd::addPointInstancerDerivatives(rdlGeometry,
                                                     pointInstancer,
                                                     pointsPAT,
                                                     motionFrames);
    } catch(const std::runtime_error &error) {
        // catch bad (inf, nan, zero) normals and derivatives
        rdlGeometry->error(error.what());
        return;
    }

    // Collect any requested primitive attributes into this
    // temporary table.   When we loop over the transforms,
    // we create a table for each instance and get the values
    // from this temp table.
    for (const auto& key : generateContext.getRequestedAttributes()) {

        // If the instance_level transform is requested then we'll
        // add it later when we get the loop over the transforms
        if (key == instanceLevelKey) {
            addInstanceTransformAttribute = true;
            continue;
        } else if (key == moonray::shading::StandardAttributes::sInstanceObjectTransform) {
            addInstanceObjectTransformAttribute = true;
            continue;
        }

        moonshine::usd::addPointInstancerPrimvar(rdlGeometry,
                                                 pointInstancer,
                                                 key,
                                                 pointsPAT,
                                                 motionFrames);
    }

    // Check that the number of indices matches the number
    // of transforms
    bool applyIndex;
    if (indices.size() == usdXformsCount) {
        applyIndex = true;
    } else if (indices.empty()) {
        applyIndex = false;
    } else {
        usdInstanceGeometry->warn("refIndices count(", indices.size(),
            ") is not equal to usdXforms/positions count(", usdXformsCount,
            "). Skip applying refIndices");
        applyIndex = false;
    }

    // Build a list of disabled indices
    std::unordered_set<int> disableIndicesSet;
    const scene_rdl2::rdl2::IntVector& disableIndices = usdInstanceGeometry->get(attrDisableIndices);
    if (!disableIndices.empty()) {
        // If mask exists (i.e. some indices are invis or deactivated), warn
        // that disable_indices is given preference
        if (!idMask.empty()) {
            usdInstanceGeometry->warn("Both disable_indices and point instancer ",
                "mask exist, using disable_indices");
        }
        for (auto &i : disableIndices) {
            if (i < static_cast<int>(usdXformsCount)) disableIndicesSet.insert(i);
        }
    } else if (!idMask.empty()) {
        for (size_t i = 0; i < idMask.size(); ++i) {
            if (i < usdXformsCount && !idMask[i]) {
                disableIndicesSet.insert(i);
            }
        }
    }

    // DisableIndicesSet contains no duplicates or elements >= usdXformsCount
    reservePrimitive(usdXformsCount - disableIndicesSet.size());

    // User rdla primitive attributes
    std::unordered_map<AttributeKey, const void*, AttributeKeyHash> attrMap;
    const scene_rdl2::rdl2::SceneObjectVector& attributes =
        usdInstanceGeometry->get(attrPrimitiveAttributes);

    for (const auto& key : generateContext.getRequestedAttributes()) {

        for (const auto& attr : attributes) {
            if (!attr->isA<scene_rdl2::rdl2::UserData>()) {
                continue;
            }
            if (attrMap.find(key) != attrMap.end()) {
                continue;
            }
            const scene_rdl2::rdl2::UserData* data = attr->asA<scene_rdl2::rdl2::UserData>();
            scene_rdl2::rdl2::AttributeType type = key.getType();
            const std::string& name = key.getName();
            if (type == scene_rdl2::rdl2::TYPE_BOOL && data->hasBoolData() &&
                name == data->getBoolKey()) {
                if (data->getBoolValues().size() == usdXformsCount) {
                    attrMap[key] = (const void*)(&data->getBoolValues());
                }
            } else if (type == scene_rdl2::rdl2::TYPE_INT && data->hasIntData() &&
                name == data->getIntKey()) {
                if (data->getIntValues().size() == usdXformsCount) {
                    attrMap[key] = (const void*)(&data->getIntValues());
                }
            } else if (type == scene_rdl2::rdl2::TYPE_FLOAT && data->hasFloatData() &&
                name == data->getFloatKey()) {
                if (data->getFloatValues().size() == usdXformsCount) {
                    attrMap[key] = (const void*)(&data->getFloatValues());
                }
            } else if (type == scene_rdl2::rdl2::TYPE_STRING && data->hasStringData() &&
                name == data->getStringKey()) {
                if (data->getStringValues().size() == usdXformsCount) {
                    attrMap[key] = (const void*)(&data->getStringValues());
                }
            } else if (type == scene_rdl2::rdl2::TYPE_RGB && data->hasColorData() &&
                name == data->getColorKey()) {
                if (data->getColorValues().size() == usdXformsCount) {
                    attrMap[key] = (const void*)(&data->getColorValues());
                }
            } else if (type == scene_rdl2::rdl2::TYPE_VEC2F && data->hasVec2fData() &&
                name == data->getVec2fKey()) {
                if (data->getVec2fValues().size() == usdXformsCount) {
                    attrMap[key] = (const void*)(&data->getVec2fValues());
                }
            } else if (type == scene_rdl2::rdl2::TYPE_VEC3F && data->hasVec3fData() &&
                name == data->getVec3fKey()) {
                if (data->getVec3fValues().size() == usdXformsCount) {
                    attrMap[key] = (const void*)(&data->getVec3fValues());
                }
            } else if (type == scene_rdl2::rdl2::TYPE_MAT4F && data->hasMat4fData() &&
                name == data->getMat4fKey()) {
                if (data->getMat4fValues().size() == usdXformsCount) {
                    attrMap[key] = (const void*)(&data->getMat4fValues());
                }
            }
        }
        if (attrMap.find(key) == attrMap.end()) {
            attrMap[key] = nullptr;
        }
    }

    size_t badXformCount = 0;
    const int maxIndex = refPrimitiveGroup.size() - 1;

    for (size_t i = 0; i < usdXformsCount; ++i) {
        if (!disableIndicesSet.empty() &&
            disableIndicesSet.find(i) != disableIndicesSet.end()) {
            continue;
        }

        int index = applyIndex ? indices[i] : 0;
        if (index > maxIndex) {
            usdInstanceGeometry->warn("Reference index ", index,
                " exceeds reference count.   Skipping instance.");
            continue;
        }

        // Skip if there is no reference geometry for this index
        if (!refPrimitiveGroup[index]) {
            continue;
        }

        // Add the usd instance transforms for each motion frame
        XformSamples xformSamples;
        xformSamples.push_back(usdMatrixToXform(usdXforms[0][i]));
        if (generateContext.isMotionBlurOn()) { 
            xformSamples.push_back(usdMatrixToXform(usdXforms[1][i]));
        }

        // Create a primitive attribute table for this instance
        PrimitiveAttributeTable instancePAT;

        // If the instance_level primitive attribute was requested
        // then convert the transform to a Mat4f and add it to the
        // primitive attribute table as a constant value.
        if (addInstanceTransformAttribute) {
            scene_rdl2::math::Mat4f instanceTransform(xformSamples[0]);
            instancePAT.addAttribute(instanceLevelKey,
                                     RATE_CONSTANT,
                                     { instanceTransform });
        }

        if (addInstanceObjectTransformAttribute) {
            scene_rdl2::math::Mat4f instanceObjectTransform(parent2render[0]);
            instancePAT.addAttribute(moonray::shading::StandardAttributes::sInstanceObjectTransform,
                                     RATE_CONSTANT,
                                     { nodeXform });
        }

        if (usdInstanceGeometry->get(attrUseReferenceAttributes)) {
            // Only adding shadow_ray_epsilon for now
            // MOONRAY-4313 - Propagate common geometry attributes to instanced primitives
            std::vector<float> refShadowRayEpsilon = { refShadowRayEpsilons[index] };
            instancePAT.addAttribute(StandardAttributes::sShadowRayEpsilon,
                                     RATE_CONSTANT,
                                     std::move(refShadowRayEpsilon));
        }

        // Add the requested primitive attributes as constant by
        // copying the value from the pointsPAT
        transferPrimitiveAttributes(pointsPAT, instancePAT, i);

        // Add explicit shading primitive attribute if it's enabled
        if (usdInstanceGeometry->get(attrExplicitShading) &&
            !addExplicitShading(rdlGeometry, instancePAT)) {
            continue;
        }

        // Add rdla primitive attributes
        for (const auto& kv : attrMap) {
            const auto& key = kv.first;
            const void* values = kv.second;
            if (values == nullptr) {
                continue;
            }
            switch (key.getType()) {
            case scene_rdl2::rdl2::TYPE_BOOL:
                instancePAT.addAttribute(
                    TypedAttributeKey<bool>(key), RATE_CONSTANT,
                    std::vector<bool>({ (*(const scene_rdl2::rdl2::BoolVector*)values)[i]}) );
                break;
            case scene_rdl2::rdl2::TYPE_INT:
                instancePAT.addAttribute(
                    TypedAttributeKey<int>(key), RATE_CONSTANT,
                    std::vector<int>({ (*(const scene_rdl2::rdl2::IntVector*)values)[i]}) );
                break;
            case scene_rdl2::rdl2::TYPE_FLOAT:
                instancePAT.addAttribute(
                    TypedAttributeKey<float>(key), RATE_CONSTANT,
                    std::vector<float>({ (*(const scene_rdl2::rdl2::FloatVector*)values)[i]}) );
                break;
            case scene_rdl2::rdl2::TYPE_STRING:
                instancePAT.addAttribute(
                    TypedAttributeKey<std::string>(key), RATE_CONSTANT,
                    {(*(const scene_rdl2::rdl2::StringVector*)values)[i]});
                break;
            case scene_rdl2::rdl2::TYPE_RGB:
                instancePAT.addAttribute(
                    TypedAttributeKey<scene_rdl2::math::Color>(key), RATE_CONSTANT,
                    {(*(const scene_rdl2::rdl2::RgbVector*)values)[i]});
                break;
            case scene_rdl2::rdl2::TYPE_VEC2F:
                instancePAT.addAttribute(
                    TypedAttributeKey<Vec2f>(key), RATE_CONSTANT,
                    {(*(const scene_rdl2::rdl2::Vec2fVector*)values)[i]});
                break;
            case scene_rdl2::rdl2::TYPE_VEC3F:
                instancePAT.addAttribute(
                    TypedAttributeKey<Vec3f>(key), RATE_CONSTANT,
                    {(*(const scene_rdl2::rdl2::Vec3fVector*)values)[i]});
                break;
            case scene_rdl2::rdl2::TYPE_MAT4F:
                instancePAT.addAttribute(
                    TypedAttributeKey<scene_rdl2::math::Mat4f>(key), RATE_CONSTANT,
                    {(*(const scene_rdl2::rdl2::Mat4fVector*)values)[i]});
                break;
            default:
                break;
            }
        }

        // Check that the transform is valid and, if so,
        // create the instance
        if (isValidXform(xformSamples)) {
            auto instance = createInstance(xformSamples,
                                           refPrimitiveGroup[index],
                                           std::move(instancePAT));

            addPrimitive(std::move(instance),
                         generateContext.getMotionBlurParams(),
                         parent2render);
        } else {
            ++badXformCount;
        }
    }

    if (badXformCount > 0) {
        usdInstanceGeometry->warn("Skipped ", badXformCount,
            " instances which contained invalid transform matrices");
    }
}

void
UsdInstanceGeometryProcedural::update(
        const UpdateContext &updateContext,
        const XformSamples &parent2render)
{}

} // namespace geom
} // namespace moonray

moonray::geom::Procedural*
UsdInstanceGeometry::createProcedural() const
{
    moonray::geom::State state;
    // Do not call state.setName here since scene_rdl2::rdl2::rdlLayer::assignmentId already
    // use rdlGeometry name. 
        
    return new moonray::geom::UsdInstanceGeometryProcedural(state);
}

void
UsdInstanceGeometry::destroyProcedural() const
{
    delete mProcedural;
}

bool
UsdInstanceGeometry::deformed() const
{
    return mProcedural->deformed();
}

void
UsdInstanceGeometry::resetDeformed()
{
    mProcedural->resetDeformed();
}

