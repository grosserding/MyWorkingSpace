cmake_minimum_required(VERSION 3.3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(SlamUeben)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include 
  /usr/include/eigen3
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

add_library(common_lib SHARED src/common/csv_utils.cpp)
add_library(LibrariesUebenLib SHARED src/lib/libraries_ueben_lib.cpp)
add_library(EKFUebenLib SHARED src/lib/ekf_ueben_lib.cpp)

add_executable(ExecuteSlamUeben src/slam_ueben_main.cpp)
add_executable(ExecuteKapitel_2_3 src/slam_in_autonomous/kapitel_2/kapitel_2_3.cpp)

add_executable(14Kap_5_3 src/vierzehn_kapiteln_des_SLAMs/kapitel_5/kapitel_5_3.cpp)

target_link_libraries(ExecuteSlamUeben LibrariesUebenLib EKFUebenLib common_lib)
target_link_libraries(ExecuteKapitel_2_3 common_lib)
target_link_libraries(14Kap_5_3 ${OpenCV_LIBS})
