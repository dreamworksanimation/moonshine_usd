// Copyright 2023 DreamWorks Animation LLC
// SPDX-License-Identifier: Apache-2.0

//

#include "Mesh.h"

#include <moonshine/geometry/data/PrimitiveUserData.h>
#include <moonray/rendering/geom/Api.h>
#include <moonray/rendering/geom/ProceduralLeaf.h>
#include <scene_rdl2/scene/rdl2/rdl2.h>
#include <scene_rdl2/common/platform/Platform.h>

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

#if PXR_MINOR_VERSION >= 21
#define GetPrimInMaster GetPrimInPrototype
#endif

#include <string>

using namespace moonray;
using namespace moonray::geom;
using namespace moonray::shading;
PXR_NAMESPACE_USING_DIRECTIVE

namespace {

void
addNormals(const UsdGeomMesh& mesh,
           PrimitiveAttributeTable& table,
           const std::vector<float>& motionFrames,
           const scene_rdl2::rdl2::Geometry *rdlGeometry)
{
    // early out if there is no normal info in mesh
    if (!mesh.GetNormalsAttr().IsAuthored()) {
        return;
    }
    std::vector<std::vector<Vec3f>> normals(motionFrames.size());

    const TfToken normalsInterpolation = mesh.GetNormalsInterpolation();
    AttributeRate attrRate = RATE_UNKNOWN;
    if (normalsInterpolation == UsdGeomTokens->vertex) {
        // default
        attrRate = RATE_VERTEX;
    } else if (normalsInterpolation == UsdGeomTokens->faceVarying) {
        attrRate = RATE_FACE_VARYING;
    } else if (normalsInterpolation == UsdGeomTokens->uniform) {
        attrRate = RATE_UNIFORM;
    } else if (normalsInterpolation == UsdGeomTokens->varying) {
        attrRate = RATE_VARYING;
    } else {
        rdlGeometry->warn("Unknown rate for normals");
        return;
    }

    const bool isLH = moonshine::usd::isLeftHanded(mesh);

    for (size_t i = 0; i < motionFrames.size(); ++i) {
        // Set the vert per face count
        VtArray<int> faceVertexCounts;
        mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts, motionFrames[i]);
        VtArray <GfVec3f> points;
        mesh.GetPointsAttr().Get(&points, motionFrames[i]);
        const uint32_t faceCount = faceVertexCounts.size();
        const uint32_t vertexCount = points.size();

        VtArray <GfVec3f> usdNormals;
        mesh.GetNormalsAttr().Get(&usdNormals, motionFrames[i]);

        if (attrRate == RATE_FACE_VARYING) {
            // 3 or 4 vertices per face
            normals[i].reserve(4 * faceCount);
            uint32_t indexOffset = 0;
            for (uint32_t fIndex = 0; fIndex < faceCount; ++fIndex) {
                uint32_t faceVertexCount = faceVertexCounts[fIndex];
                for (uint32_t vIndex = 0; vIndex < faceVertexCount; ++vIndex) {
                    uint32_t realVIndex = isLH ? (faceVertexCount - vIndex - 1) : vIndex;
                    normals[i].emplace_back(
                        usdNormals[indexOffset + realVIndex][0],
                        usdNormals[indexOffset + realVIndex][1],
                        usdNormals[indexOffset + realVIndex][2]);
                }
                indexOffset += faceVertexCount;
            }
        } else if (attrRate == RATE_VARYING) {
            normals[i].reserve(vertexCount);
            for (uint32_t vId = 0; vId < vertexCount; ++vId) {
                normals[i].emplace_back(
                    usdNormals[vId][0],
                    usdNormals[vId][1],
                    usdNormals[vId][2]);
            }
        } else if (attrRate == RATE_VERTEX) {
            normals[i].reserve(vertexCount);
            for (uint32_t vId = 0; vId < vertexCount; ++vId) {
                normals[i].emplace_back(
                    usdNormals[vId][0],
                    usdNormals[vId][1],
                    usdNormals[vId][2]);
            }
        } else if (attrRate == RATE_UNIFORM) {
            normals[i].reserve(faceCount);
            for (uint32_t fIndex = 0; fIndex < faceCount; ++fIndex) {
                normals[i].emplace_back(
                    usdNormals[fIndex][0],
                    usdNormals[fIndex][1],
                    usdNormals[fIndex][2]);
            }
        }
    }

    table.addAttribute(StandardAttributes::sNormal, attrRate, std::move(normals));
}

template<typename PrimvarType>
void
addUVsByType(const UsdGeomPrimvar& uvPrimvar,
             const UsdGeomMesh& mesh,
             float frame,
             PrimitiveAttributeTable& table,
             const scene_rdl2::rdl2::Geometry *rdlGeometry)
{
    const TfToken uvInterpolation = uvPrimvar.GetInterpolation();
    AttributeRate attrRate = RATE_UNKNOWN;
    if (uvInterpolation == UsdGeomTokens->vertex) {
        // default
        attrRate = RATE_VERTEX;
    } else if (uvInterpolation == UsdGeomTokens->faceVarying) {
        attrRate = RATE_FACE_VARYING;
    } else if (uvInterpolation == UsdGeomTokens->uniform) {
        attrRate = RATE_UNIFORM;
    } else if (uvInterpolation == UsdGeomTokens->varying) {
        attrRate = RATE_VARYING;
    } else {
        rdlGeometry->warn("Unknown rate for UVs");
        return;
    }

    // Set the vert per face count
    VtArray<int> faceVertexCounts;
    mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts, frame);
    VtArray <GfVec3f> points;
    mesh.GetPointsAttr().Get(&points, frame);
    const uint32_t faceCount = faceVertexCounts.size();
    const uint32_t vertexCount = points.size();

    VtArray <PrimvarType> usdUvs;
    uvPrimvar.Get(&usdUvs, frame);

    const bool isLH = moonshine::usd::isLeftHanded(mesh);

    std::vector<Vec2f> textureUV;
    if (attrRate == RATE_FACE_VARYING) {
        // Check if the UVs are indexed
        bool isIndexed = false;
        VtArray<int> uvIndices;
        if (uvPrimvar.GetIndicesAttr().HasValue()) {
            uvPrimvar.GetIndicesAttr().Get(&uvIndices, frame);
            isIndexed = true;
        }

        // 3 or 4 vertices per face
        textureUV.reserve(4 * faceCount);
        uint32_t indexOffset = 0;
        for (uint32_t fIndex = 0; fIndex < faceCount; ++fIndex) {
            uint32_t faceVertexCount = faceVertexCounts[fIndex];
            for (uint32_t vIndex = 0; vIndex < faceVertexCount; ++vIndex) {
                uint32_t realVIndex = isLH ? (faceVertexCount - vIndex - 1) : vIndex;
                const uint32_t uvIndex = (isIndexed) ? uvIndices[indexOffset + realVIndex] :
                                                       indexOffset + realVIndex;
                textureUV.emplace_back(
                    usdUvs[uvIndex][0],
                    usdUvs[uvIndex][1]);
            }
            indexOffset += faceVertexCount;
        }
    } else if (attrRate == RATE_VARYING) {
        textureUV.reserve(vertexCount);
        for (uint32_t vId = 0; vId < vertexCount; ++vId) {
            textureUV.emplace_back(
                usdUvs[vId][0],
                usdUvs[vId][1]);
        }
    } else if (attrRate == RATE_VERTEX) {
        textureUV.reserve(vertexCount);
        for (uint32_t vId = 0; vId < vertexCount; ++vId) {
            textureUV.emplace_back(
                usdUvs[vId][0],
                usdUvs[vId][1]);
        }
    } else if (attrRate == RATE_UNIFORM) {
        textureUV.reserve(faceCount);
        for (uint32_t fIndex = 0; fIndex < faceCount; ++fIndex) {
            textureUV.emplace_back(
                usdUvs[fIndex][0],
                usdUvs[fIndex][1]);
        }
    }

    table.addAttribute(StandardAttributes::sSurfaceST, attrRate, std::move(textureUV));
}

void
addUVs(const UsdGeomMesh& mesh,
       PrimitiveAttributeTable& table,
       float frame,
       const scene_rdl2::rdl2::Geometry *rdlGeometry)
{
    // early out if there is no uv info in mesh
    const UsdGeomPrimvarsAPI pvAPI(mesh.GetPrim());

    // "st" by default, fallback to "uv"
    const bool hasST = pvAPI.HasPrimvar(TfToken("st"));
    if (!hasST && !pvAPI.HasPrimvar(TfToken("uv"))) {
        return;
    }
    const UsdGeomPrimvar uvPrimvar = hasST ? 
        pvAPI.GetPrimvar(TfToken("st")) : pvAPI.GetPrimvar(TfToken("uv"));

    const TfToken& primvarType = uvPrimvar.GetTypeName().GetAsToken();
    if (primvarType == TfToken("float2[]") || primvarType == TfToken("texCoord2f[]")) {
        addUVsByType<GfVec2f>(uvPrimvar,
                              mesh,
                              frame,
                              table,
                              rdlGeometry);
    } else if (primvarType == TfToken("float3[]") || primvarType == TfToken("texCoord3f[]")) {
        addUVsByType<GfVec3f>(uvPrimvar,
                              mesh,
                              frame,
                              table,
                              rdlGeometry);
    }
}

} // end anonymous namespace

namespace moonshine {
namespace usd {

std::unique_ptr<Primitive>
createMesh(const scene_rdl2::rdl2::Geometry *rdlGeometry,
           const GenerateContext &generateContext,
           std::vector<float>& motionFrames,
           const UsdGeomMesh &mesh,
           const scene_rdl2::rdl2::Layer *rdlLayer,
           const CommonAttrs& commonAttrs,
           const MeshAttrs& meshAttrs)
{
    const float currentFrame = commonAttrs.useEvaluationFrame ?
                               commonAttrs.evaluationFrame :
                               generateContext.getCurrentFrame();

    // motionFrames may be updated here, depending on discovered mbType
    const scene_rdl2::rdl2::MotionBlurType mbType =
        getMotionBlurType<UsdGeomMesh>(mesh,
                                       commonAttrs.motionBlurType,
                                       motionFrames,
                                       currentFrame);

    PrimitiveAttributeTable primitiveAttributeTable;

    VertexBuffer<Vec3fa, InterleavedTraits> vertices =
        getVertices<UsdGeomMesh>(rdlGeometry,
                                 mesh,
                                 primitiveAttributeTable,
                                 mbType,
                                 motionFrames);

    // Set the vert per face count
    VtArray<int> faceVertexCounts;
    mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts, currentFrame);
    PolygonMesh::FaceVertexCount faceVertexCount(faceVertexCounts.begin(),
                                                 faceVertexCounts.end());
    const size_t faceCount = faceVertexCount.size();

    // Store the vert indicies in order of face list to build the mesh
    const bool isLH = isLeftHanded(mesh);

    VtArray<int> faceVertexIndices;
    mesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices,
                                        currentFrame);

    PolygonMesh::IndexBuffer indices;
    int indexOffset = 0;
    for (size_t fIndex = 0; fIndex < faceCount; ++fIndex) {
        size_t faceVertexCount = faceVertexCounts[fIndex];
        faceVertexCounts.push_back(faceVertexCount);
        for (size_t vIndex = 0; vIndex < faceVertexCount; ++vIndex) {
            // reverse winding order for left handed geometry
            size_t realVIndex = isLH ? (faceVertexCount - vIndex - 1) : vIndex;
            indices.push_back(faceVertexIndices[indexOffset + realVIndex]);
        }
        indexOffset += faceVertexCount;
    }

    // check if mesh is valid - if not, early-out
    if (vertices.empty() || faceVertexCount.empty() || indices.empty()) {
        return nullptr;
    }

    // per face assignment id
    std::vector<UsdGeomSubset> subsets;
    if (mesh.GetPrim().IsInstanceProxy()) {
        subsets = UsdGeomSubset::GetAllGeomSubsets(UsdGeomMesh(mesh.GetPrim().GetPrimInMaster()));
    } else {
        subsets = UsdGeomSubset::GetAllGeomSubsets(mesh);
    }

    const std::string meshName = mesh.GetPrim().GetName();

    std::vector<int> facesetPartIndices(subsets.size(), -1);
    int meshPartIndex = -1;
    const std::vector<std::string>& geomPartList = commonAttrs.partList;

    const int meshAssignmentId = rdlLayer->getAssignmentId(rdlGeometry, meshName);
    PolygonMesh::FaceToPartBuffer faceToPart(faceCount, 0);
    std::vector<int> faceAssignmentIds(faceCount, meshAssignmentId);
    int partCount = 1;
    if (!subsets.empty()) {
        partCount += subsets.size();
        int subsetIndex = 0;
        for (const UsdGeomSubset &subset: subsets) {

            VtArray<int> subsetIndices;
            subset.GetIndicesAttr().Get(&subsetIndices, currentFrame);
            const scene_rdl2::rdl2::String &partName = subset.GetPrim().GetName().GetString();
            const int partFaceCount = subsetIndices.size();
            const int partAssignmentId = rdlLayer->getAssignmentId(rdlGeometry, partName);
            for (const int i: subsetIndices) {
                faceAssignmentIds[i] = partAssignmentId;
                faceToPart[i] = subsetIndex;
            }

            for (size_t j = 0; j < geomPartList.size(); j++) {
                if (partName == geomPartList[j]) {
                    facesetPartIndices[subsetIndex] = j;
                    break;
                }
            }

            ++subsetIndex;
        }

        // Is the mesh itself named in the parts list?  Get the index.
        for (size_t i = 0; i < geomPartList.size(); ++i) {
            if (meshName == geomPartList[i]) {
                meshPartIndex = i;
                break;
            }
        }
    }
    LayerAssignmentId layerAssignmentId(std::move(faceAssignmentIds));

    const scene_rdl2::rdl2::Geometry::SideType sideType = rdlGeometry->getSideType();
    bool usdDoubleSided;
    mesh.GetDoubleSidedAttr().Get(&usdDoubleSided, currentFrame);
    const bool singleSided =
        (sideType == scene_rdl2::rdl2::Geometry::SideType::SINGLE_SIDED) ||
        (sideType == scene_rdl2::rdl2::Geometry::SideType::MESH_DEFAULT_SIDED && !usdDoubleSided);

    TfToken subdivisonScheme;
    mesh.GetSubdivisionSchemeAttr().Get(&subdivisonScheme,
                                        currentFrame);

    const bool isPolymesh = meshAttrs.subdType == SubdType::FORCE_POLYGONMESH ||
                            (meshAttrs.subdType == SubdType::USE_MESH_TYPE &&
                            subdivisonScheme == UsdGeomTokens->none);

    if (isPolymesh) {
        addNormals(mesh,
                   primitiveAttributeTable,
                   motionFrames,
                   rdlGeometry);
    }

    addUVs(mesh,
           primitiveAttributeTable,
           currentFrame,
           rdlGeometry);

    // Add any primitive attribute requested for shading
    for (const auto& key : generateContext.getRequestedAttributes()) {
        addPrimvar(rdlGeometry,
                   mesh,
                   key,
                   primitiveAttributeTable,
                   motionFrames);
    }

    // Populate the primitive attribute table with any user data
    const scene_rdl2::rdl2::PrimitiveAttributeFrame primitiveAttributeFrame = commonAttrs.primitiveAttributeFrame;
    bool useFirstFrame = (primitiveAttributeFrame != scene_rdl2::rdl2::PrimitiveAttributeFrame::SECOND_MOTION_STEP);
    bool useSecondFrame = (primitiveAttributeFrame != scene_rdl2::rdl2::PrimitiveAttributeFrame::FIRST_MOTION_STEP);
    geometry::processMeshUserData(commonAttrs.primitiveAttributes,
                                  facesetPartIndices,
                                  meshPartIndex,
                                  useFirstFrame,
                                  useSecondFrame,
                                  primitiveAttributeTable);

    // Add explicit shading primitive attribute if it's enabled
    if (commonAttrs.explicitShading &&
        !addExplicitShading(rdlGeometry, primitiveAttributeTable)) {

        return nullptr;
    }

    // build the primitive
    std::unique_ptr<Primitive> result;
    if (isPolymesh) {

        // we don't remove unassigned faces from subdivision control meshes
        // (but we do for polygon mesh) since it will change the topology
        // of subivision mesh, and affect the limit surface. The unassigned
        // face in subdivision mesh won't get evaluated during tessellation.
        removeUnassignedFaces(rdlLayer,
                              layerAssignmentId,
                              faceToPart,
                              faceVertexCount,
                              indices,
                              &primitiveAttributeTable);

        if (indices.empty()) {
            return nullptr;
        }

        std::unique_ptr<PolygonMesh> primitive =
            createPolygonMesh(std::move(faceVertexCount),
                              std::move(indices),
                              std::move(vertices),
                              std::move(layerAssignmentId),
                              std::move(primitiveAttributeTable));

        // set additional primitive attributes
        primitive->setName(meshName);
        primitive->setIsSingleSided(singleSided);
        primitive->setParts(partCount, std::move(faceToPart));
        primitive->setMeshResolution(meshAttrs.meshResolution);
        primitive->setAdaptiveError(meshAttrs.adaptiveError);
        primitive->setIsNormalReversed(rdlGeometry->getReverseNormals());
        primitive->setSmoothNormal(meshAttrs.smoothNormal);
        primitive->setCurvedMotionBlurSampleCount(commonAttrs.curvedMotionBlurSampleCount);
        result.reset(primitive.release());
    } else {
        SubdivisionMesh::Scheme scheme = (subdivisonScheme == UsdGeomTokens->bilinear) ?
                                         SubdivisionMesh::Scheme::BILINEAR :
                                         SubdivisionMesh::Scheme::CATMULL_CLARK;

        std::unique_ptr<SubdivisionMesh> primitive =
            createSubdivisionMesh(scheme,
                                  std::move(faceVertexCount),
                                  std::move(indices),
                                  std::move(vertices),
                                  std::move(layerAssignmentId),
                                  std::move(primitiveAttributeTable));

        // set additional primitive attributes
        primitive->setName(meshName);
        primitive->setIsSingleSided(singleSided);
        primitive->setParts(partCount, std::move(faceToPart));
        primitive->setMeshResolution(meshAttrs.meshResolution);
        primitive->setAdaptiveError(meshAttrs.adaptiveError);
        primitive->setModifiability(Primitive::Modifiability::DEFORMABLE);
        primitive->setIsNormalReversed(rdlGeometry->getReverseNormals());
        primitive->setCurvedMotionBlurSampleCount(commonAttrs.curvedMotionBlurSampleCount);
        result.reset(primitive.release());
    }

    return result;
}

} // namespace usd
} // namespace moonshine

