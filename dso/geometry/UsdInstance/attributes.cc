// Copyright 2023 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#include <scene_rdl2/scene/rdl2/rdl2.h>

RDL2_DSO_ATTR_DECLARE

    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::String>       attrStage;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::String>       attrPointInstancerPath;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseStageCache;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseEvaluationFrame;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Float>        attrEvaluationFrame;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Int>          attrInstanceLevel;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::IntVector>    attrDisableIndices;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseReferenceXforms;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseReferenceAttributes;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::SceneObjectVector> attrPrimitiveAttributes;

    DECLARE_COMMON_MOTION_BLUR_ATTRIBUTES

RDL2_DSO_ATTR_DEFINE(scene_rdl2::rdl2::Geometry)

    attrStage =
        sceneClass.declareAttribute<scene_rdl2::rdl2::String>("stage", "", scene_rdl2::rdl2::FLAGS_FILENAME);
    sceneClass.setMetadata(attrStage, "label", "stage");
    sceneClass.setMetadata(attrStage, "comment", "USD Stage to load\n");

    attrPointInstancerPath =
        sceneClass.declareAttribute<scene_rdl2::rdl2::String>("point_instancer_path", "");
    sceneClass.setMetadata(attrPointInstancerPath, "label", "point instancer path");
    sceneClass.setMetadata(attrPointInstancerPath, "comment", "The point instancer to load from the USD Stage\n");

    attrUseStageCache =
    sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_stage_cache", false, { "use stage cache" });
    sceneClass.setMetadata(attrUseStageCache, "label", "use stage cache");
    sceneClass.setMetadata(attrUseStageCache, "comment",
        "Load the entire stage and use StageCache to share it among all UsdInstanceGeometry objects.\n"
        "If this is false, load a stage masked to the prim path just for this UsdInstanceGeometry.\n"
        "For large stages with thousands of unique assets, it is faster to enable the stage cache\n");

    attrUseEvaluationFrame =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_evaluation_frame", false, { "use evaluation frame" });
    sceneClass.setMetadata(attrUseEvaluationFrame, "label", "use evaluation frame");
    sceneClass.setMetadata(attrUseEvaluationFrame, "comment",
        "uses \"evaluation frame\" instead of SceneVariables frame\n");

    attrEvaluationFrame =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Float>("evaluation_frame", 1, { "evaluation frame" });
    sceneClass.setMetadata(attrEvaluationFrame, "label", "evaluation frame");
    sceneClass.setMetadata(attrEvaluationFrame, "comment",
        "evaluate geometry at specified frame instead of SceneVariables frame\n");

    attrInstanceLevel =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Int>("instance_level", 0,
        scene_rdl2::rdl2::FLAGS_ENUMERABLE);
    sceneClass.setEnumValue(attrInstanceLevel, 0, "instance level 0");
    sceneClass.setEnumValue(attrInstanceLevel, 1, "instance level 1");
    sceneClass.setEnumValue(attrInstanceLevel, 2, "instance level 2");
    sceneClass.setEnumValue(attrInstanceLevel, 3, "instance level 3");
    sceneClass.setEnumValue(attrInstanceLevel, 4, "instance level 4");
    sceneClass.setMetadata(attrInstanceLevel, "comment", "Level of this instance.  This level can be referenced in TransformSpaceMap to allow for transforming data to/from the local space of each instance this instancer produces.");

    attrDisableIndices =
        sceneClass.declareAttribute<scene_rdl2::rdl2::IntVector>("disable_indices");
    sceneClass.setMetadata(attrDisableIndices, "label", "disable indices");
    sceneClass.setMetadata(attrDisableIndices, "comment",
        "A list of index values to hide / disable. "
        "For example, with 4 instances you can supply a list of 0, 2 to disable those instances. "
        "If an index in this list is out of range, it is ignored.");

    attrUseReferenceXforms =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_reference_xforms", true);
    sceneClass.setMetadata(attrUseReferenceXforms, "label", "use reference xforms");
    sceneClass.setMetadata(attrUseReferenceXforms, "comment", "Transform the reference (prototype) geometry by it's node_xform parameter before applying the instance transform");

    attrUseReferenceAttributes =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_reference_attributes", true);
    sceneClass.setMetadata(attrUseReferenceAttributes, "label", "use reference attributes");
    sceneClass.setMetadata(attrUseReferenceAttributes, "comment", "Use the geometry attributes of the reference (prototype) instead of the ones on the InstanceGeometry.   Currently only works for shadow_ray_epsilon");

    attrPrimitiveAttributes =
        sceneClass.declareAttribute<scene_rdl2::rdl2::SceneObjectVector>(
        "primitive_attributes", scene_rdl2::rdl2::SceneObjectVector(), scene_rdl2::rdl2::FLAGS_NONE,
        scene_rdl2::rdl2::INTERFACE_USERDATA, { "primitive attributes" });
    sceneClass.setMetadata(attrPrimitiveAttributes, "label", "primitive attributes");
    sceneClass.setMetadata(attrPrimitiveAttributes, "comment",
        "A list of UserData to specify arbitrary primitive attributes"
        "(For example, color or roughness multiplier) per -instance");


    DEFINE_COMMON_MOTION_BLUR_ATTRIBUTES

RDL2_DSO_ATTR_END

