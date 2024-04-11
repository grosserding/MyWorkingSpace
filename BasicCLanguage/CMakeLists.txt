cmake_minimum_required(VERSION 2.8)

project(BasicCLanguage)

include_directories(include)
add_executable(ExecuteLeetcode src/leetcode.cpp)
add_executable(ExecuteFreiUeben src/freiUeben.cpp)
