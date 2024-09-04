cmake_minimum_required(VERSION 3.3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)
# set(G2O_ROOT /usr/local/include/g2o)
list(APPEND CMAKE_MODULE_PATH ~/DingSpace/GithubProject/g2o/cmake_modules)

project(SlamUeben)
find_package(absl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono thread)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(G2O REQUIRED)
include_directories(
  include
  /usr/include/eigen3
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

# link_libraries(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(common_lib SHARED src/common/csv_utils.cpp)
add_library(LibrariesUebenLib SHARED src/lib/libraries_ueben_lib.cpp)
add_library(EKFUebenLib SHARED src/lib/ekf_ueben_lib.cpp)

add_executable(ExecuteSlamUeben src/slam_ueben_main.cpp)
add_executable(ExecuteKapitel_2_3
               src/slam_in_autonomous/kapitel_2/kapitel_2_3.cpp)

add_executable(14Kap_3_2 src/vierzehn_kapiteln_des_SLAMs/kapitel_3/3.2.cpp)
add_executable(14Kap_3_6 src/vierzehn_kapiteln_des_SLAMs/kapitel_3/3.6.cpp)
add_executable(14Kap_5_3
               src/vierzehn_kapiteln_des_SLAMs/kapitel_5/kapitel_5_3.cpp)
add_executable(14Kap_5_4
               src/vierzehn_kapiteln_des_SLAMs/kapitel_5/kapitel_5_4.cpp)
add_executable(14Kap_6_3 src/vierzehn_kapiteln_des_SLAMs/kapitel_6/6.3.cpp)
add_executable(14Kap_6_3_self
               src/vierzehn_kapiteln_des_SLAMs/kapitel_6/6.3_self.cpp)
add_executable(14Kap_6_4 src/vierzehn_kapiteln_des_SLAMs/kapitel_6/6.4.cpp)
add_executable(calcTransform
               src/vierzehn_kapiteln_des_SLAMs/kapitel_6/calcTransform.cpp)
add_executable(pcl_icp src/vierzehn_kapiteln_des_SLAMs/kapitel_6/pcl_icp.cpp)
add_executable(14Kap_7_2_extract_feature
               src/vierzehn_kapiteln_des_SLAMs/kap_7/7.2.cpp)
add_executable(14Kap_7_4_pose_estimation_2d2d
               src/vierzehn_kapiteln_des_SLAMs/kap_7/7.4.cpp)
add_executable(14Kap_7_8_1_3d2d_EPnP
               src/vierzehn_kapiteln_des_SLAMs/kap_7/7.8.1.cpp)
add_executable(14Kap_7_8_2_3d2d_BA
               src/vierzehn_kapiteln_des_SLAMs/kap_7/7.8.2.cpp)
add_executable(vorstellung src/vorstellung.cpp)

target_link_libraries(ExecuteSlamUeben LibrariesUebenLib EKFUebenLib common_lib)
target_link_libraries(ExecuteKapitel_2_3 common_lib)
target_link_libraries(14Kap_5_3 ${OpenCV_LIBS})
target_link_libraries(14Kap_5_4 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBS})
target_link_libraries(14Kap_6_3 ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(14Kap_6_3_self ${OpenCV_LIBS} ${CERES_LIBRARIES}
                      ${Sophus_LIBS})
target_link_libraries(calcTransform ${OpenCV_LIBS} ${CERES_LIBRARIES}
                      ${Sophus_LIBS})
target_link_libraries(14Kap_6_4 ${OpenCV_LIBS} ${G2O_CORE_LIBRARY}
                      ${G2O_STUFF_LIBRARY})
target_link_libraries(pcl_icp ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(14Kap_7_2_extract_feature ${OpenCV_LIBS})
target_link_libraries(14Kap_7_4_pose_estimation_2d2d ${OpenCV_LIBS})
target_link_libraries(14Kap_7_8_1_3d2d_EPnP ${OpenCV_LIBS})
target_link_libraries(14Kap_7_8_2_3d2d_BA ${OpenCV_LIBS} ${G2O_CORE_LIBRARY}
                      ${G2O_STUFF_LIBRARY})
