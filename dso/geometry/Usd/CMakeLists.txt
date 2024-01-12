# Copyright 2023-2024 DreamWorks Animation LLC
# SPDX-License-Identifier: Apache-2.0

moonray_dso_simple(UsdGeometry
    DEPENDENCIES
        Moonray::rendering_geom
        ${PROJECT_NAME}::geometry_usd
        SceneRdl2::common_platform
        SceneRdl2::scene_rdl2
        usd)

if(CMAKE_CXX_COMPILER_ID STREQUAL Intel)
    target_compile_options(UsdGeometry
        PRIVATE
            -Wno-deprecated                 # Usd uses deprecated headers.
        )
endif()
