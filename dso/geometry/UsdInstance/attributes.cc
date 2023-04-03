// Copyright 2023 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#include <scene_rdl2/scene/rdl2/rdl2.h>

RDL2_DSO_ATTR_DECLARE

    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::String>       attrStage;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::String>       attrPointInstancerPath;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseStageCache;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Int>          attrInstanceLevel;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::IntVector>    attrDisableIndices;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseReferenceXforms;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseReferenceAttributes;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::SceneObjectVector> attrPrimitiveAttributes;

    DECLARE_COMMON_MOTION_BLUR_ATTRIBUTES
    DECLARE_COMMON_EVALUATION_FRAME_ATTRIBUTES

RDL2_DSO_ATTR_DEFINE(scene_rdl2::rdl2::Geometry)

    attrStage =
        sceneClass.declareAttribute<scene_rdl2::rdl2::String>("stage", "", scene_rdl2::rdl2::FLAGS_FILENAME);
    sceneClass.setMetadata(attrStage, "label", "stage");
    sceneClass.setMetadata(attrStage, "comment", "USD Stage to load containing the point instancer");
    sceneClass.setGroup("USD", attrStage);

    attrPointInstancerPath =
        sceneClass.declareAttribute<scene_rdl2::rdl2::String>("point_instancer_path", "");
    sceneClass.setMetadata(attrPointInstancerPath, "label", "point instancer path");
    sceneClass.setMetadata(attrPointInstancerPath, "comment", "The point instancer to load from the USD Stage");
    sceneClass.setGroup("USD", attrPointInstancerPath);

    attrUseStageCache =
    sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_stage_cache", false, { "use stage cache" });
    sceneClass.setMetadata(attrUseStageCache, "label", "use stage cache");
    sceneClass.setMetadata(attrUseStageCache, "comment",
        "Load the entire stage and use StageCache to share it among all UsdInstanceGeometry objects.\n"
        "If this is false, load a stage masked to the prim path just for this UsdInstanceGeometry.\n"
        "For large stages with thousands of unique assets, it is faster to enable the stage cache.");
    sceneClass.setGroup("USD", attrUseStageCache);

    attrInstanceLevel =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Int>("instance_level", 0,
        scene_rdl2::rdl2::FLAGS_ENUMERABLE);
    sceneClass.setEnumValue(attrInstanceLevel, 0, "instance level 0");
    sceneClass.setEnumValue(attrInstanceLevel, 1, "instance level 1");
    sceneClass.setEnumValue(attrInstanceLevel, 2, "instance level 2");
    sceneClass.setEnumValue(attrInstanceLevel, 3, "instance level 3");
    sceneClass.setEnumValue(attrInstanceLevel, 4, "instance level 4");
    sceneClass.setMetadata(attrInstanceLevel, "comment", "Sets the level/depth of this instance.  This adds a "
            "Mat4f primitive attribute to the geometry which can be referenced during shading to use the local "
            "space of each instance.  The name of the primitive attribute corresponds the the instance level "
            " that is set (i.e. \"instance_level_0\", \"instance_level_1\", etc)");
    sceneClass.setGroup("Instancing", attrInstanceLevel);

    attrDisableIndices =
        sceneClass.declareAttribute<scene_rdl2::rdl2::IntVector>("disable_indices");
    sceneClass.setMetadata(attrDisableIndices, "label", "disable indices");
    sceneClass.setMetadata(attrDisableIndices, "comment",
        "A list of index values to hide / disable. "
        "For example, with 4 instances you can supply a list of 0, 2 to disable those instances. "
        "If an index in this list is out of range, it is ignored.");
    sceneClass.setGroup("Instancing", attrDisableIndices);

    attrUseReferenceXforms =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_reference_xforms", true);
    sceneClass.setMetadata(attrUseReferenceXforms, "label", "use reference xforms");
    sceneClass.setMetadata(attrUseReferenceXforms, "comment", "Transform the reference (prototype) geometry by it's node_xform parameter before applying the instance transform");
    sceneClass.setGroup("Instancing", attrUseReferenceXforms);

    attrUseReferenceAttributes =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_reference_attributes", true);
    sceneClass.setMetadata(attrUseReferenceAttributes, "label", "use reference attributes");
    sceneClass.setMetadata(attrUseReferenceAttributes, "comment", "Use the geometry attributes of the reference (prototype) instead of the ones on the InstanceGeometry.   Currently only works for shadow_ray_epsilon");
    sceneClass.setGroup("Instancing", attrUseReferenceAttributes);

    attrPrimitiveAttributes =
        sceneClass.declareAttribute<scene_rdl2::rdl2::SceneObjectVector>(
        "primitive_attributes", scene_rdl2::rdl2::SceneObjectVector(), scene_rdl2::rdl2::FLAGS_NONE,
        scene_rdl2::rdl2::INTERFACE_USERDATA, { "primitive attributes" });
    sceneClass.setMetadata(attrPrimitiveAttributes, "label", "primitive attributes");
    sceneClass.setMetadata(attrPrimitiveAttributes, "comment",
        "A list of UserData to specify arbitrary primitive attributes"
        "(For example, color or roughness multiplier) per-instance");
    sceneClass.setGroup("User Data", attrPrimitiveAttributes);


    DEFINE_COMMON_MOTION_BLUR_ATTRIBUTES
    DEFINE_COMMON_EVALUATION_FRAME_ATTRIBUTES

RDL2_DSO_ATTR_END

