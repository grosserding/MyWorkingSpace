cmake_minimum_required(VERSION 2.8)

project(LibraryUeben)
find_package(Sophus REQUIRED)
include_directories(include /usr/include/eigen3 ${Sophus_INCLUDE_DIRS})

add_library(LibEigenUeben SHARED src/lib/eigen_ueben_lib.cpp)

add_executable(ExecuteEigenUeben src/eigen_ueben_main.cpp)

target_link_libraries(ExecuteEigenUeben LibEigenUeben)
