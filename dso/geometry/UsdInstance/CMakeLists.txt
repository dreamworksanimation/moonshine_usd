# Copyright 2023-2024 DreamWorks Animation LLC
# SPDX-License-Identifier: Apache-2.0

moonray_dso_simple(UsdInstanceGeometry
    DEPENDENCIES
        Moonray::rendering_shading
        ${PROJECT_NAME}::geometry_usd
        SceneRdl2::common_platform
        SceneRdl2::scene_rdl2
        usd)

if(CMAKE_CXX_COMPILER_ID STREQUAL Intel)
    target_compile_options(UsdInstanceGeometry
        PRIVATE
            -Wno-deprecated                 # Usd uses deprecated headers.
        )
endif()
