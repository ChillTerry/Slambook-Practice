find_package(Sophus REQUIRED)
include_directories("/usr/include/eigen3")
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap Sophus::Sophus ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
