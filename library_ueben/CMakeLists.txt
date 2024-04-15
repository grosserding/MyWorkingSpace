cmake_minimum_required(VERSION 3.3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(LibraryUeben)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
include_directories(include /usr/include/eigen3 ${Sophus_INCLUDE_DIRS})

add_library(LibrariesUebenLib SHARED src/lib/libraries_ueben_lib.cpp)

add_executable(ExecuteLibrariesUeben src/libraries_ueben_main.cpp)

target_link_libraries(ExecuteLibrariesUeben LibrariesUebenLib ${FMT_LIBRARIES})
