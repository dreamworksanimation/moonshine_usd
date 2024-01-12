# Copyright 2023-2024 DreamWorks Animation LLC
# SPDX-License-Identifier: Apache-2.0

set(component geometry_usd)

set(installIncludeDir ${PACKAGE_NAME}/geometry/usd)
set(exportGroup ${PROJECT_NAME}Targets)

add_library(${component} SHARED "")
add_library(${PROJECT_NAME}::${component} ALIAS ${component})

target_sources(${component}
    PRIVATE
        Common.cc
        Curves.cc
        Mesh.cc
        Points.cc
)

set_property(TARGET ${component}
    PROPERTY PUBLIC_HEADER
        Usd.h
        Mesh.h
        Curves.h
        Points.h
        Common.h
)

target_include_directories(${component}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${component}
    PUBLIC
        Moonray::rendering_geom
        Moonray::rendering_shading
        Moonshine::geometry_data
        SceneRdl2::common_platform
        SceneRdl2::scene_rdl2
        usd
        usdSkel

# these libraries are required by USD, but we don't
# currently have a USD config with the dependencies linked
        Python::Module
        Boost::headers
        TBB::tbb)

# If at Dreamworks add a SConscript stub file so others can use this library.
SConscript_Stub(${component})

# Set standard compile/link options
MoonshineUsd_cxx_compile_definitions(${component})
MoonshineUsd_cxx_compile_features(${component})
MoonshineUsd_cxx_compile_options(${component})
MoonshineUsd_link_options(${component})

# -------------------------------------
# Install the target and the export set
# -------------------------------------
include(GNUInstallDirs)

# install the target
install(TARGETS ${component}
    COMPONENT ${component}
    EXPORT ${exportGroup}
    LIBRARY
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        NAMELINK_SKIP
    RUNTIME
        DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${installIncludeDir}
)

# # install the export set
# install(
#     EXPORT ${exportGroup}
#     NAMESPACE ${PROJECT_NAME}::
#     DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION}
# )
