cmake_minimum_required(VERSION 2.8.3)
project(edge_extractor)

find_package(catkin REQUIRED
  srrg_cmake_modules)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(OpenCV)
find_package(cv_bridge)
catkin_package(
  CATKIN_DEPENDS sensor_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(
  ${cv_bridge_INCLUDE_DIRS}
)
add_executable(edge_extractor 
  src/edge_extractor.cpp)

target_link_libraries(edge_extractor ${OpenCV_LIBRARIES})
target_link_libraries(edge_extractor ${cv_bridge_LIBRARIES})
target_link_libraries(edge_extractor ${catkin_LIBRARIES})
