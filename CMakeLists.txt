# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

EXECUTE_PROCESS(COMMAND git describe --tags --abbrev=0
        TIMEOUT 5
        OUTPUT_VARIABLE GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )
MESSAGE(STATUS "building from git tag ${GIT_VERSION}")
add_definitions(-DCONFIG_FIRMWARE_VERSION=\"${GIT_VERSION}\")

# if(DEFINED ENV{HOMEKIT_PATH})
#   set(HOMEKIT_PATH $ENV{HOMEKIT_PATH})
# else()
#   set(HOMEKIT_PATH ${CMAKE_CURRENT_LIST_DIR}/esp-homekit-sdk)
# endif(DEFINED ENV{HOMEKIT_PATH})
# 配网用到esp-homekti-sdk，届时需要拷贝一份esp-homekit-sdk或者设置$HOMEKIT_PATH

# set(IOT_SOLUTION_PATH  ${CMAKE_CURRENT_LIST_DIR}/esp-iot-solution)
# include(${IOT_SOLUTION_PATH}/component.cmake)

# set(EXTRA_COMPONENT_DIRS ${HOMEKIT_PATH}/components/homekit ${HOMEKIT_PATH}/examples/common ${IOT_SOLUTION_PATH}/components/button)

set(EXTRA_COMPONENT_DIRS ${CMAKE_SOURCE_DIR}/lib/hi229 ${CMAKE_SOURCE_DIR}/lib/libtcp ${CMAKE_SOURCE_DIR}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(imu-esp-node)

#idf_build_set_property(COMPILE_OPTIONS "-O3" APPEND)