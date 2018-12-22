# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(GLM_INSTALL_ENABLE OFF CACHE BOOL "Flag to override default GLM_INSTALL_ENABLE value")
set(GLM_TEST_ENABLE OFF CACHE BOOL "Flag to override default GLM_TEST_ENABLE value")
add_compile_definitions(GLM_FORCE_SILENT_WARNINGS)

if (NOT DEFINED GLM_ROOT_DIR)
    set(GLM_ROOT_DIR
        ${ARION_ROOT}/Arion/third_party/glm
        CACHE STRING "Path to GLM root directory")
endif()
