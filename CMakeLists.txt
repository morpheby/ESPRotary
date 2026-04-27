cmake_minimum_required(VERSION 3.16)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    project(ESPRotary CXX)
endif()

add_library(esprotary STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ESPRotary.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CH32VEncoder.cpp"
)
target_include_directories(esprotary PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
