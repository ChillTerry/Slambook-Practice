cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories("/usr/include/eigen3/")

add_executable(ORB_CV ORB_CV.cpp)
target_link_libraries(ORB_CV ${OpenCV_LIBS})

add_executable(ORB_Self ORB_Self.cpp)
target_link_libraries(ORB_Self ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d g2o_core g2o_stuff ${OpenCV_LIBS} Sophus::Sophus)

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d g2o_core g2o_stuff ${OpenCV_LIBS} Sophus::Sophus)