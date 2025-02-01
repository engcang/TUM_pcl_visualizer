cmake_minimum_required(VERSION 3.1.0)
project(tum_pcl_visualizer)

### Set compiler
ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -fexceptions -g -ggdb -O3")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall ${CMAKE_CXX_FLAGS}")

### Get packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  nav_msgs
  pcl_ros
  tf
  tf_conversions
  livox_ros_driver
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(PCL REQUIRED)

### Get packages headers
include_directories(
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

catkin_package()

#### Build
add_executable(${PROJECT_NAME}_node src/main_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${PCL_LIBRARIES} yaml-cpp)
