cmake_minimum_required(VERSION 3.5)

set(PROJECT_VER 0.0.0.1)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(ESP32-2432S028)

add_compile_definitions( LV_CONF_INCLUDE_SIMPLE=1 )
idf_build_set_property(COMPILE_OPTIONS "-I../main" APPEND)

idf_build_set_property(COMPILE_OPTIONS "-D_PROJECT_NAME_=\"${CMAKE_PROJECT_NAME}\"" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-D_PROJECT_VER_=\"${PROJECT_VER}\"" APPEND)
