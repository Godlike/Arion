# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required(VERSION 3.0)
cmake_policy(VERSION 3.0)

project(ArionTests)

include_directories(${ARION_INCLUDE_DIR} ${CATCH_ROOT}/single_include/catch2)

function(arion_add_test)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCE ARGS DEPENDS)
    cmake_parse_arguments(
        arion_add_test
        "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
    )

    add_executable(
        ${arion_add_test_NAME}_test
        ${arion_add_test_SOURCE}
    )

    if (arion_add_test_DEPENDS)
        target_link_libraries(${arion_add_test_NAME}_test ${arion_add_test_DEPENDS})
    endif()

    add_test(
        NAME ${arion_add_test_NAME}
        COMMAND ${arion_add_test_NAME}_test ${arion_add_test_ARGS}
    )
endfunction()

arion_add_test(NAME CollisionDetection
    SOURCE Detection.cpp
    DEPENDS ${ARION_LIB}
)
