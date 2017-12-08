# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required(VERSION 3.1)

project(Arion)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(ARION_BUILD_DEMOS "Build Arion demo." ON)
option(ARION_BUILD_TESTS "Build Arion tests." ON)

message(STATUS "${PROJECT_NAME} configuration:")
message(STATUS "-- ARION_BUILD_DEMOS: ${ARION_BUILD_DEMOS}")
message(STATUS "-- ARION_BUILD_TESTS: ${ARION_BUILD_DEMOS}")

set(ARION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Arion root directory.")

include(ArionConfig)

if (NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
endif()

if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
endif()

if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output)
endif()

add_subdirectory(Arion)

if (${ARION_BUILD_DEMOS})
    add_subdirectory(demo)
endif()

if (ARION_BUILD_TESTS)
    enable_testing()
    set(CATCH_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Catch2)
    add_subdirectory(test)
endif()