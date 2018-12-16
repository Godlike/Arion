# Copyright (C) 2017-2018 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

include(ArionConfig)

project(${ARION_NAME})

set(ARION_LIB_ALIAS Arion::Collision)

add_library(${PROJECT_NAME} "")
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
add_library(${ARION_LIB_ALIAS} ALIAS ${PROJECT_NAME})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

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
)
set(ARION_SOURCES
    sources/DebugImplementation.cpp
    sources/Intersection.cpp
    sources/Shape.cpp
    sources/GilbertJohnsonKeerthi.cpp
    sources/SimpleShapeIntersectionDetector.cpp
    sources/BoundingVolumes.cpp
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

target_sources(${PROJECT_NAME}
    PRIVATE
        ${ARION_HEADERS}
        ${ARION_SOURCES}
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        ${ARION_DEFINITIONS}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${ARION_INCLUDE_DIR}
)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
        VERSION ${ARION_VERSION}
        SOVERSION ${ARION_SOVERSION}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Epona::Math
)

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

install( TARGETS ${PROJECT_NAME}
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
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${ARION_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake.gen

    @ONLY
)

file(GENERATE
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake.gen
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${ARION_INSTALL_LIBRARY_DIR}
    COMPONENT arion_headers
)
