cmake_minimum_required(VERSION 3.3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(SlamUeben)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
include_directories(include /usr/include/eigen3 ${Sophus_INCLUDE_DIRS})

add_library(LibrariesUebenLib SHARED src/lib/libraries_ueben_lib.cpp)
add_library(EKFUebenLib SHARED src/lib/ekf_ueben_lib.cpp)

add_executable(ExecuteSlamUeben src/slam_ueben_main.cpp)

add_executable(ExecuteKapitelZwei src/slam_in_autonomous/kapitel_zwei.cpp)

target_link_libraries(ExecuteSlamUeben LibrariesUebenLib EKFUebenLib)
