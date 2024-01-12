// Copyright 2023-2024 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//  UsdGeometry is a procedural that can generate one or more Moonray
//  primitives from a specified geometry Prim on a USD stage.


#include "attributes.cc"

#include <moonshine_usd/geometry/usd/Usd.h>

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
#include <pxr/usd/usdUtils/stageCache.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/points.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdSkel/root.h>

#if PXR_MINOR_VERSION >= 21
#define GetPrimInMaster GetPrimInPrototype
#endif

using namespace moonray;
using namespace moonray::geom;
using namespace moonray::shading;

RDL2_DSO_CLASS_BEGIN(UsdGeometry, scene_rdl2::rdl2::Geometry)

public:
    RDL2_DSO_DEFAULT_CTOR(UsdGeometry)
    moonray::geom::Procedural* createProcedural() const;
    void destroyProcedural() const;
    bool deformed() const;
    void resetDeformed();

RDL2_DSO_CLASS_END(UsdGeometry)

namespace {

const scene_rdl2::rdl2::String sDefaultPartName("");

// Get the transform sample from a usd
// geometry primitive for each motion frame
template<typename T>
XformSamples
getXformSamples(const std::vector<float>& motionFrames,
                const bool useMasterXform,
                const float currentFrame,
                const UsdPrim& prim,
                const T& geomPrim)
{
    XformSamples rdlXformSamples;
    for (const auto frame: motionFrames) {
        GfMatrix4d gfXform;
        if (useMasterXform && prim.IsInstanceProxy()) {
            UsdGeomMesh masterMesh(prim.GetPrimInMaster());
            gfXform = masterMesh.ComputeLocalToWorldTransform(currentFrame);
        } else {
            UsdSkelRoot skelRoot = UsdSkelRoot::Find(prim);
            if (skelRoot) {
                // if this is part of a skeleton, compute xform from the skel root
                gfXform = skelRoot.ComputeLocalToWorldTransform(currentFrame);
            } else {
                gfXform = geomPrim.ComputeLocalToWorldTransform(currentFrame);
            }
        }

        rdlXformSamples.emplace_back(
            gfXform[0][0], gfXform[0][1], gfXform[0][2],
            gfXform[1][0], gfXform[1][1], gfXform[1][2],
            gfXform[2][0], gfXform[2][1], gfXform[2][2],
            gfXform[3][0], gfXform[3][1], gfXform[3][2]
        );
    }
    return rdlXformSamples;
}

// Get the motion blur frames from the geometry context
void
getMotionFrames(const GenerateContext& generateContext,
                const float currentFrame,
                std::vector<float>& motionFrames)
{
    if (generateContext.isMotionBlurOn()) {
        for (const auto motionStep : generateContext.getMotionSteps()) {
                const float sampleNumber = scene_rdl2::rdl2::computeSampleNumber(
                    currentFrame, motionStep);
            motionFrames.push_back(sampleNumber);
        }
    } else {
        motionFrames.push_back(currentFrame);
    }
}

// Get the mesh resolution based on whether the
// "enable_max_geometry_resolution" scene variable
// is set to true
float
getMeshResolution(const scene_rdl2::rdl2::Geometry *rdlGeometry,
                  float inputRes)
{
    const scene_rdl2::rdl2::SceneVariables& vars =
        rdlGeometry->getSceneClass().getSceneContext()->getSceneVariables();

    const bool enableMaxRes = vars.get(scene_rdl2::rdl2::SceneVariables::sEnableMaxGeomResolution);
    const int maxRes = vars.get(scene_rdl2::rdl2::SceneVariables::sMaxGeomResolution);
    return enableMaxRes ? std::min(inputRes, float(maxRes)) : inputRes;
}

}  // anonymous namespace

namespace moonray {
namespace geom {

PXR_NAMESPACE_USING_DIRECTIVE

class UsdGeometryProcedural : public ProceduralLeaf
{
public:
    UsdGeometryProcedural(const State &state)
        : ProceduralLeaf(state)
    {}

    void generate(const GenerateContext &generateContext,
            const XformSamples &parent2render) override;

    void update(const UpdateContext &updateContext,
            const XformSamples &parent2render) override;

    // Add the moonray primitive to the scene.  If
    // "use_prim_xform" is true then get the primitive's
    // transform and create a moonray TransformedPrimitive.
    // If the primitive is an instance and "use_master_xform"
    // is true, then use the transform from the master prim.
    template<typename T>
    void
    addMoonrayPrimitive(std::unique_ptr<Primitive>& primitive,
                        const UsdPrim& prim,
                        const T& geomType,
                        const bool usePrimXform,
                        const bool useMasterXform,
                        const std::vector<float>& motionFrames,
                        const float currentFrame,
                        const GenerateContext& generateContext,
                        const XformSamples& parent2render)
    {
        if (!primitive) {
            return;
        }

        if (usePrimXform) {
            XformSamples rdlXformSamples =
                getXformSamples<T>(motionFrames,
                                   useMasterXform,
                                   currentFrame,
                                   prim,
                                   geomType);

            std::unique_ptr<TransformedPrimitive> xformPrim =
                createTransformedPrimitive(rdlXformSamples,
                                           std::move(primitive));

            addPrimitive(std::move(xformPrim),
                         generateContext.getMotionBlurParams(),
                         parent2render);
        } else {
            addPrimitive(std::move(primitive),
                         generateContext.getMotionBlurParams(),
                         parent2render);
        }
    }

    // Create a moonray mesh primitive from a usd
    // primitive and add it to the scene.
    void
    addMeshPrimitive(const UsdGeomMesh& mesh,
                     const UsdPrim& prim,
                     const scene_rdl2::rdl2::Geometry *rdlGeometry,
                     const scene_rdl2::rdl2::Layer *rdlLayer,
                     const GenerateContext &generateContext,
                     const XformSamples& parent2render,
                     std::vector<float>& motionFrames,
                     const float currentFrame,
                     const ::moonshine::usd::CommonAttrs& commonAttrs,
                     const bool usePrimXform,
                     const bool useMasterXform,
                     const float meshResolution,
                     const float adaptiveError,
                     const int subdType,
                     const bool useRotationalMotionBlur,
                     const bool smoothNormal)
    {
        ::moonshine::usd::MeshAttrs meshAttrs;
        meshAttrs.meshResolution = getMeshResolution(rdlGeometry, meshResolution);
        meshAttrs.adaptiveError = adaptiveError;
        meshAttrs.subdType = static_cast<::moonshine::usd::SubdType>(subdType);
        meshAttrs.smoothNormal = smoothNormal;

        std::unique_ptr<Primitive> primitive =
            ::moonshine::usd::createMesh(rdlGeometry,
                                         generateContext,
                                         motionFrames,
                                         mesh,
                                         rdlLayer,
                                         commonAttrs,
                                         meshAttrs);

        if (primitive == nullptr) {
            return;
        }

        addMoonrayPrimitive<UsdGeomMesh>(primitive,
                                         prim,
                                         mesh,
                                         usePrimXform,
                                         useMasterXform,
                                         motionFrames,
                                         currentFrame,
                                         generateContext,
                                         parent2render);
    }

    // Create a moonray curves primitive from a usd
    // primitive and add it to the scene.
    void
    addCurvesPrimitive(const UsdGeomBasisCurves& curves,
                       const UsdPrim& prim,
                       const scene_rdl2::rdl2::Geometry *rdlGeometry,
                       const scene_rdl2::rdl2::Layer *rdlLayer,
                       const GenerateContext &generateContext,
                       const XformSamples& parent2render,
                       std::vector<float>& motionFrames,
                       const float currentFrame,
                       const ::moonshine::usd::CommonAttrs& commonAttrs,
                       const bool usePrimXform,
                       const bool useMasterXform,
                       const float radiusMult,
                       const float baseWidthFactor,
                       const float tipWidthFactor,
                       const int curvedMotionBlurSampleCount)
    {
        std::unique_ptr<Primitive> primitive =
            ::moonshine::usd::createCurves(rdlGeometry,
                                           generateContext,
                                           motionFrames,
                                           curves,
                                           rdlLayer,
                                           commonAttrs,
                                           radiusMult,
                                           baseWidthFactor,
                                           tipWidthFactor,
                                           curvedMotionBlurSampleCount);

        if (primitive == nullptr) {
            rdlGeometry->error("Curves primitive is nullptr");
            return;
        }

        addMoonrayPrimitive<UsdGeomBasisCurves>(primitive,
                                                prim,
                                                curves,
                                                usePrimXform,
                                                useMasterXform,
                                                motionFrames,
                                                currentFrame,
                                                generateContext,
                                                parent2render);
    }

    // Create a moonray curves primitive from a usd
    // primitive and add it to the scene.
    void
    addPointsPrimitive(const UsdGeomPoints& points,
                       const UsdPrim& prim,
                       const scene_rdl2::rdl2::Geometry *rdlGeometry,
                       const scene_rdl2::rdl2::Layer *rdlLayer,
                       const GenerateContext &generateContext,
                       const XformSamples& parent2render,
                       std::vector<float>& motionFrames,
                       const float currentFrame,
                       const ::moonshine::usd::CommonAttrs& commonAttrs,
                       const bool usePrimXform,
                       const bool useMasterXform,
                       const float pointScale,
                       const int curvedMotionBlurSampleCount)
    {
        std::unique_ptr<Primitive> primitive =
            ::moonshine::usd::createPoints(rdlGeometry,
                                           generateContext,
                                           motionFrames,
                                           points,
                                           rdlLayer,
                                           commonAttrs,
                                           pointScale,
                                           curvedMotionBlurSampleCount);

        if (primitive == nullptr) {
            rdlGeometry->error("Points primitive is nullptr");
            return;
        }

        addMoonrayPrimitive<UsdGeomPoints>(primitive,
                                           prim,
                                           points,
                                           usePrimXform,
                                           useMasterXform,
                                           motionFrames,
                                           currentFrame,
                                           generateContext,
                                           parent2render);
    }

    void
    addUsdPrim(const UsdPrim& prim,
               const scene_rdl2::rdl2::Geometry *rdlGeometry,
               const scene_rdl2::rdl2::Layer *rdlLayer,
               const GenerateContext &generateContext,
               const XformSamples& parent2render,
               std::vector<float>& motionFrames,
               const float currentFrame,
               const ::moonshine::usd::CommonAttrs& commonAttrs,
               const UsdGeometry *usdGeometry)
    {
        if (UsdGeomMesh mesh{prim}) {
            const bool useRotationalMotionBlur = usdGeometry->get(attrUseRotationMotionBlur);
            float adaptiveError = useRotationalMotionBlur ? 0.0f : usdGeometry->get(attrAdaptiveError);
            addMeshPrimitive(mesh, prim,
                             rdlGeometry, rdlLayer,
                             generateContext,
                             parent2render,
                             motionFrames, currentFrame,
                             commonAttrs,
                             usdGeometry->get(attrUsePrimXform),
                             usdGeometry->get(attrUseMasterXform),
                             usdGeometry->get(attrMeshResolution),
                             adaptiveError,
                             usdGeometry->get(attrSubdType),
                             useRotationalMotionBlur,
                             usdGeometry->get(attrSmoothNormal));
        } else if (UsdGeomBasisCurves curves{prim}) {
            addCurvesPrimitive(curves, prim,
                               rdlGeometry, rdlLayer,
                               generateContext,
                               parent2render,
                               motionFrames, currentFrame,
                               commonAttrs,
                               usdGeometry->get(attrUsePrimXform),
                               usdGeometry->get(attrUseMasterXform),
                               usdGeometry->get(attrRadiusMult),
                               usdGeometry->get(attrBaseWidthFactor),
                               usdGeometry->get(attrTipWidthFactor),
                               usdGeometry->get(attrCurvedMotionBlurSampleCount));
        } else if (UsdGeomPoints points{prim}) {
            addPointsPrimitive(points, prim,
                               rdlGeometry, rdlLayer,
                               generateContext,
                               parent2render,
                               motionFrames, currentFrame,
                               commonAttrs,
                               usdGeometry->get(attrUsePrimXform),
                               usdGeometry->get(attrUseMasterXform),
                               usdGeometry->get(attrRadiusMult),
                               usdGeometry->get(attrCurvedMotionBlurSampleCount));
        }
    }

    void
    addUsdPrims(const UsdPrim& prim,
                const scene_rdl2::rdl2::Geometry *rdlGeometry,
                const scene_rdl2::rdl2::Layer *rdlLayer,
                const GenerateContext &generateContext,
                const XformSamples& parent2render,
                std::vector<float>& motionFrames,
                const float currentFrame,
                const ::moonshine::usd::CommonAttrs& commonAttrs,
                const UsdStageRefPtr& stage,
                const UsdGeometry *usdGeometry,
                int hierarchyLevel)
    {
        if (UsdGeomXform xform{prim}) {

            // TODO: Can a UsdGeomXform be an instance proxy?
            UsdPrim parentPrim = prim;
            if (prim.IsInstanceProxy()) {
                parentPrim = prim.GetPrimInMaster();
            }

            hierarchyLevel++;

            // prim.GetChildren() returns nothing for an instance proxy!
            for (const UsdPrim& child: parentPrim.GetChildren()) {

                UsdPrim childPrim = child;
                if (prim.IsInstanceProxy()) {
                    const TfToken& nameToken = child.GetPath().GetNameToken();
                    const SdfPath& primPath = prim.GetPath().AppendChild(nameToken);
                    childPrim = stage->GetPrimAtPath(primPath);
                }

                addUsdPrims(childPrim,
                           rdlGeometry,
                           rdlLayer,
                           generateContext,
                           parent2render,
                           motionFrames,
                           currentFrame,
                           commonAttrs,
                           stage,
                           usdGeometry,
                           hierarchyLevel);
            }
        } else {
            addUsdPrim(prim,
                       rdlGeometry,
                       rdlLayer,
                       generateContext,
                       parent2render,
                       motionFrames,
                       currentFrame,
                       commonAttrs,
                       usdGeometry);
        }
    }
};

void
UsdGeometryProcedural::generate(
        const GenerateContext& generateContext,
        const XformSamples &parent2render)
{
    const scene_rdl2::rdl2::Geometry *rdlGeometry = generateContext.getRdlGeometry();

    const UsdGeometry *usdGeometry =
        static_cast<const UsdGeometry*>(rdlGeometry);

    const std::string& stageFile = usdGeometry->get(attrStage);
    if (stageFile.empty()) {
        rdlGeometry->error("\"stage\" parameter is empty");
        return;
    }

    const std::string& primPath = usdGeometry->get(attrPrimPath);
    if (primPath.empty()) {
        rdlGeometry->error("\"prim_path\" parameter is empty");
        return;
    }

    UsdStageRefPtr stage;
    if (usdGeometry->get(attrUseStageCache)) {
        UsdStageCacheContext stageCacheContext(UsdUtilsStageCache::Get());
        stage = UsdStage::Open(stageFile);
    } else {
        UsdStagePopulationMask mask;
        mask.Add(SdfPath(primPath));
        stage = UsdStage::OpenMasked(stageFile, mask);
    }

    if (!stage) {
        rdlGeometry->error("Could not open stage: ", stageFile);
        return;
    }

    UsdPrim prim = stage->GetPrimAtPath(SdfPath(primPath));
    if (!prim) {
        rdlGeometry->error("Could not find prim path: ", primPath);
        return;
    }

    Curves::SubType curvesSubType = Curves::SubType::UNKNOWN;
    switch (rdlGeometry->get(attrCurvesSubType)) {
    case 0:
        curvesSubType = Curves::SubType::RAY_FACING;
        break;
    case 1:
        curvesSubType = Curves::SubType::ROUND;
        break;
    case 2:
        curvesSubType = Curves::SubType::NORMAL_ORIENTED;
        break;
    default:
        rdlGeometry->warn("Unknown curve subtype, defaulting to ray facing.");
        curvesSubType = Curves::SubType::RAY_FACING;
    }

    // Get attribute common to all geometry types
    ::moonshine::usd::CommonAttrs commonAttrs;
    commonAttrs.useEvaluationFrame = usdGeometry->get(attrUseEvaluationFrame);
    commonAttrs.evaluationFrame = usdGeometry->get(attrEvaluationFrame);
    commonAttrs.useRotationMotionBlur = usdGeometry->get(attrUseRotationMotionBlur);
    commonAttrs.motionBlurType = static_cast<scene_rdl2::rdl2::MotionBlurType>(usdGeometry->get(attrMotionBlurType));
    commonAttrs.curvedMotionBlurSampleCount = usdGeometry->get(attrCurvedMotionBlurSampleCount);
    commonAttrs.partList = usdGeometry->get(attrPartList);
    commonAttrs.primitiveAttributes = usdGeometry->get(attrPrimitiveAttributes);
    commonAttrs.primitiveAttributeFrame =
        static_cast<scene_rdl2::rdl2::PrimitiveAttributeFrame>(usdGeometry->get(attrPrimitiveAttributeFrame));
    commonAttrs.curvesSubType = curvesSubType;
    commonAttrs.tessellationRate = usdGeometry->get(attrTessellationRate);
    commonAttrs.explicitShading = usdGeometry->get(attrExplicitShading);

    const scene_rdl2::rdl2::Layer *rdlLayer = generateContext.getRdlLayer();
    const float currentFrame = generateContext.getCurrentFrame();

    std::vector<float> motionFrames;
    getMotionFrames(generateContext,
                    currentFrame,
                    motionFrames);

    int hierarchyLevel = 0;
    addUsdPrims(prim,
                rdlGeometry,
                rdlLayer,
                generateContext,
                parent2render,
                motionFrames,
                currentFrame,
                commonAttrs,
                stage,
                usdGeometry,
                hierarchyLevel);
}

void
UsdGeometryProcedural::update(
        const UpdateContext &updateContext,
        const XformSamples &parent2render)
{}

} // namespace geom
} // namespace moonray

moonray::geom::Procedural*
UsdGeometry::createProcedural() const
{
    moonray::geom::State state;
    // Do not call state.setName here since scene_rdl2::rdl2::rdlLayer::assignmentId already
    // use rdlGeometry name.

    return new moonray::geom::UsdGeometryProcedural(state);
}

void
UsdGeometry::destroyProcedural() const
{
    delete mProcedural;
}

bool
UsdGeometry::deformed() const
{
    return mProcedural->deformed();
}

void
UsdGeometry::resetDeformed()
{
    mProcedural->resetDeformed();
}

