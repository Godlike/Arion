# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

include(ArionConfig)

project(${ARION_NAME})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

if (NOT GLM_FOUND)
    add_subdirectory(${GLM_ROOT_DIR})
endif()

if (NOT EPONA_FOUND)
    option(EPONA_BUILD_DEMOS "Build Epona demo." OFF)
    option(EPONA_BUILD_TESTS "Build Epona tests." OFF)
    add_subdirectory(${ARION_ROOT}/Arion/godlike/Epona)
endif()

#Target
include_directories(
    ${ARION_INCLUDE_DIR}
)

message(STATUS "Version: ${ARION_VERSION}")

set(ARION_HEADERS
    include/Arion/Debug.hpp
    include/Arion/debug/DebugDummy.hpp
    include/Arion/debug/DebugImplementation.hpp
    include/Arion/Intersection.hpp
    include/Arion/Shape.hpp
    include/Arion/ConfigurationSpaceObject.hpp
    include/Arion/GilbertJohnsonKeerthi.hpp
    include/Arion/ExpandingPolytopeAlgorithm.hpp
    include/Arion/SimpleShapeIntersection.hpp
    include/Arion/SimpleShapeIntersectionDetector.hpp
    include/Arion/BoundingVolumes.hpp
    include/Arion/RayCasting.hpp
)
set(ARION_SOURCES
    sources/DebugImplementation.cpp
    sources/Intersection.cpp
    sources/Shape.cpp
    sources/ConfigurationSpaceObject.cpp
    sources/GilbertJohnsonKeerthi.cpp
    sources/SimpleShapeIntersectionDetector.cpp
    sources/BoundingVolumes.cpp
    sources/RayCasting.cpp
)

if (MSVC)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/ArionVersionInfo.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/ArionVersionInfo.rc

        @ONLY
    )

    set(ARION_EXTRA
        ${ARION_EXTRA}
        ${CMAKE_CURRENT_BINARY_DIR}/ArionVersionInfo.rc
    )
endif()

add_library(${ARION_LIB}
    ${ARION_HEADERS}
    ${ARION_SOURCES}
)

target_compile_definitions(${ARION_LIB} PUBLIC ${ARION_DEFINITIONS})

target_include_directories(${ARION_LIB} PUBLIC
    ${EPONA_INCLUDE_DIR}
)

set_target_properties(
    ${ARION_LIB}
    PROPERTIES
        VERSION ${ARION_VERSION}
        SOVERSION ${ARION_SOVERSION}
)

target_link_libraries(${ARION_LIB} ${EPONA_LIB})

install( DIRECTORY ${GLM_INCLUDE_DIR}/glm
    DESTINATION ${ARION_INSTALL_INCLUDE_DIR}
    COMPONENT arion_extra
    FILES_MATCHING
        PATTERN "*.hpp"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
        PATTERN "*.h"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
        PATTERN "*.inl"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

install( TARGETS ${ARION_LIB}
    COMPONENT arion_libraries
    RUNTIME DESTINATION ${ARION_INSTALL_LIBRARY_DIR}
    LIBRARY DESTINATION ${ARION_INSTALL_LIBRARY_DIR}
    ARCHIVE DESTINATION ${ARION_INSTALL_LIBRARY_DIR}
)

install( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${ARION_INSTALL_INCLUDE_DIR}
    COMPONENT arion_headers
    FILES_MATCHING PATTERN "*.hpp"
    PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ArionCollisionConfigVersion.cmake
    VERSION ${ARION_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/ArionCollisionConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ArionCollisionConfig.cmake

    @ONLY
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/ArionCollisionConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/ArionCollisionConfigVersion.cmake
    DESTINATION ${ARION_INSTALL_LIBRARY_DIR}
    COMPONENT arion_headers
)
