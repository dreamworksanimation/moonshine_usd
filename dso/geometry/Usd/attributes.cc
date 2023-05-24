// Copyright 2023 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#include <scene_rdl2/scene/rdl2/rdl2.h>

RDL2_DSO_ATTR_DECLARE

    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::String>       attrStage;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::String>       attrPrimPath;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::String>       attrMask;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUsePrimXform;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseMasterXform;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Bool>         attrUseStageCache;

    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Int>          attrSubdType;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Float>        attrRadiusMult;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Float>        attrBaseWidthFactor;
    scene_rdl2::rdl2::AttributeKey<scene_rdl2::rdl2::Float>        attrTipWidthFactor;

    DECLARE_COMMON_USER_DATA_ATTRIBUTES
    DECLARE_COMMON_CURVES_ATTRIBUTES
    DECLARE_COMMON_MESH_ATTRIBUTES
    DECLARE_COMMON_MOTION_BLUR_ATTRIBUTES
    DECLARE_COMMON_EVALUATION_FRAME_ATTRIBUTES
    DECLARE_COMMON_EXPLICIT_SHADING_ATTRIBUTES

RDL2_DSO_ATTR_DEFINE(scene_rdl2::rdl2::Geometry)

    attrStage =
        sceneClass.declareAttribute<scene_rdl2::rdl2::String>("stage", "", scene_rdl2::rdl2::FLAGS_FILENAME);
    sceneClass.setMetadata(attrStage, "label", "stage");
    sceneClass.setMetadata(attrStage, "comment", "USD Stage to load");
    sceneClass.setGroup("USD", attrStage);

    attrPrimPath =
        sceneClass.declareAttribute<scene_rdl2::rdl2::String>("prim_path", "");
    sceneClass.setMetadata(attrPrimPath, "label", "prim path");
    sceneClass.setMetadata(attrPrimPath, "comment", "The geometry Prim to load from the USD Stage");
    sceneClass.setGroup("USD", attrPrimPath);

    attrUsePrimXform =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_prim_xform", true, { "use prim xform" });
    sceneClass.setMetadata(attrUsePrimXform, "label", "use prim xform");
    sceneClass.setMetadata(attrUsePrimXform, "comment",
        "Include the xform from the Prim during geometry creation");
    sceneClass.setGroup("USD", attrUsePrimXform);

    attrUseMasterXform =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_master_xform", false, { "use master xform" });
    sceneClass.setMetadata(attrUseMasterXform, "label", "use master xform");
    sceneClass.setMetadata(attrUseMasterXform, "comment",
        "If the prim is an instance proxy, use the xform from the master Prim during geometry creation");
    sceneClass.setGroup("USD", attrUseMasterXform);

    attrUseStageCache =
    sceneClass.declareAttribute<scene_rdl2::rdl2::Bool>("use_stage_cache", false, { "use stage cache" });
    sceneClass.setMetadata(attrUseStageCache, "label", "use stage cache");
    sceneClass.setMetadata(attrUseStageCache, "comment",
        "Load the entire stage and use StageCache to share it among all UsdGeometry objects.\n"
        "If this is false, load a stage masked to the prim path just for this UsdGeometry.\n"
        "For large stages with thousands of unique assets, it is faster to enable the stage cache");
    sceneClass.setGroup("USD", attrUseStageCache);

    attrSubdType =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Int>("subd_type", 0,
            scene_rdl2::rdl2::FLAGS_ENUMERABLE, scene_rdl2::rdl2::INTERFACE_GENERIC, { "subd type" });
    sceneClass.setMetadata(attrSubdType, "label", "subd type");
    sceneClass.setEnumValue(attrSubdType, 0, "use mesh type");
    sceneClass.setEnumValue(attrSubdType, 1, "force polygon mesh");
    sceneClass.setEnumValue(attrSubdType, 2, "force subdivision mesh");
    sceneClass.setMetadata(attrSubdType, "comment",
        "PolygonMesh/SubdivisionMesh prim type to create.\n"
        "\"use mesh type\" will use the type the Mesh prim specifies.\n"
        "\"force polygon mesh\" will always resolve to PolygonMesh.\n"
        "\"force subdivision mesh\" will always resolve to SubdivisionMesh.");
    sceneClass.setGroup("Mesh", attrSubdType);

    attrRadiusMult =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Float>("radius_mult", 1.0f, { "radius mult", "point scale", "point_scale" });
    sceneClass.setMetadata(attrRadiusMult, "label", "radius mult");
    sceneClass.setMetadata(attrRadiusMult, "comment",
        "Multiplier for the radius of points and curves");
    sceneClass.setGroup("Curve", attrRadiusMult);

    attrBaseWidthFactor =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Float>("base_width_factor", 1.0f, { "base width factor" });
    sceneClass.setMetadata(attrBaseWidthFactor, "label", "base width factor");
    sceneClass.setMetadata(attrBaseWidthFactor, "comment",
            "Multiplier for the radius of the base of curves");
    sceneClass.setGroup("Curve", attrBaseWidthFactor);

    attrTipWidthFactor =
        sceneClass.declareAttribute<scene_rdl2::rdl2::Float>("tip_width_factor", 1.0f, { "tip width factor" });
    sceneClass.setMetadata(attrTipWidthFactor, "label", "tip width factor");
    sceneClass.setMetadata(attrTipWidthFactor, "comment",
            "Multiplier for the radius of the tip of curves");
    sceneClass.setGroup("Curve", attrTipWidthFactor);

    DEFINE_COMMON_USER_DATA_ATTRIBUTES
    DEFINE_COMMON_CURVES_ATTRIBUTES
    DEFINE_COMMON_MESH_ATTRIBUTES
    DEFINE_COMMON_MOTION_BLUR_ATTRIBUTES
    DEFINE_COMMON_EVALUATION_FRAME_ATTRIBUTES
    DEFINE_COMMON_EXPLICIT_SHADING_ATTRIBUTES


RDL2_DSO_ATTR_END

