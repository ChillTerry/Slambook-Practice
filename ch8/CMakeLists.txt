cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    "/usr/include/eigen3",
    ${OpenCV_INCLUDE_DIRS},
    ${G2O_INCLUDE_DIRS},
    ${Sophus_INCLUDE_DIRS},
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})

