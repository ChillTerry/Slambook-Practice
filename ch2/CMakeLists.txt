cmake_minimum_required(VERSION 2.8)

project(Hello)

add_executable(hello hello.cpp)

add_library(hello_lib hello_lib.cpp)

add_executable(use_lib use_lib.cpp)

target_link_libraries(use_lib hello_lib)