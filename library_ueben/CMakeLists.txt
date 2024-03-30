cmake_minimum_required(VERSION 2.8)

project(LibraryUeben)

include_directories(include)

add_library(EigenUebenLib SHARED src/lib/eigen_ueben_lib.cpp)

add_executable(EigenUebenExecute src/eigen_ueben_main.cpp)

target_link_libraries(EigenUebenExecute EigenUebenLib)
