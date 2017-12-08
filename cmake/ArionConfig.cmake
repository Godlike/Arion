# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(ARION_NAME "ArionCollision" CACHE STRING "Arion project name.")

if(NOT DEFINED ARION_ROOT)
    set(ARION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Arion root directory.")
endif()

list(APPEND CMAKE_MODULE_PATH "${ARION_ROOT}/Arion/cmake")

#Build flags
if (UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Werror -pedantic")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s -O3")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -ggdb3 -O0")
elseif(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14 /MP /W3")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

#GLM
include(GlmConfig)
find_package(GLM)

set(ARION_INCLUDE_DIR
    ${ARION_ROOT}
    ${ARION_ROOT}/Arion/include
    ${GLM_INCLUDE_DIR}
    CACHE LIST "Arion include directories."
)

if (NOT DEFINED INSTALL_INCLUDE_DIR)
    set(INSTALL_INCLUDE_DIR "include/godlike" CACHE STRING "Path to directory holding headers")
endif()

if (NOT DEFINED INSTALL_LIBRARY_DIR)
    set(INSTALL_LIBRARY_DIR "lib" CACHE STRING "Path to directory holding libraries")
endif()

set(ARION_LIB ${ARION_NAME} CACHE STRING "Name of Arion Collision library")
set(ARION_LIB_FULL ${ARION_LIB}.dll CACHE STRING "Full name of Arion Collision library")

set(ARION_INSTALL_INCLUDE_DIR ${INSTALL_INCLUDE_DIR})
set(ARION_INSTALL_LIBRARY_DIR ${INSTALL_LIBRARY_DIR}/${ARION_NAME})

set(ARION_VENDOR "Godlike")
set(ARION_DESCRIPTION "Arion Collision library")
set(ARION_COMMENT "")
set(ARION_COPYRIGHT "Copyright (C) 2017 by Godlike")
set(ARION_LEGAL_TM "Distributed under the MIT license (http://opensource.org/licenses/MIT)")

set(ARION_VERSION_MAJOR 0)
set(ARION_VERSION_MINOR 1)
set(ARION_VERSION_PATCH 0)
set(ARION_VERSION_TWEAK 0)

set(ARION_VERSION "${ARION_VERSION_MAJOR}.${ARION_VERSION_MINOR}.${ARION_VERSION_PATCH}")
set(ARION_SOVERSION "${ARION_VERSION_MAJOR}.${ARION_VERSION_MINOR}")

if (BUILD_SHARED_LIBS)
    add_definitions(-DARION_SHARED)
endif()
