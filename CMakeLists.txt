cmake_minimum_required(VERSION 2.8.3)
project(ground_segmentation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")



find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  velodyne_pointcloud
)


catkin_package(
)

include_directories(


  ${catkin_INCLUDE_DIRS}
)

add_executable(ground_segmentation src/ground_segmentation_main.cpp src/ground_segmentation.cpp)

target_link_libraries(ground_segmentation ${catkin_LIBRARIES})
