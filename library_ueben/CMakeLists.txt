cmake_minimum_required( VERSION 2.8 )

project(LibraryUeben)

add_executable( EigenUebenExecute src/eigen_ueben_main.cpp)
add_library(EigenUebenLib src/lib/eigen_ueben_lib.cpp)