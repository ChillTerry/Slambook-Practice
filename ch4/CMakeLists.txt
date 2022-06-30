cmake_minimum_required(VERSION 2.8)
project(useSophus)

find_package(Sophus REQUIRED)
include_directories("/usr/include/eigen3")
add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)
