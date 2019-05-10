cmake_minimum_required(VERSION 2.8.3)
project(mvcam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)

find_package(OpenCV)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)

link_directories(
 lib
 ${catkin_LIB_DIRS} 
)

add_executable(mvcam src/mvcam.cpp)
target_link_libraries(mvcam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} MVSDK)

# add_executable(mvcam1 src/mvcam1.cpp)
# target_link_libraries(mvcam1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} mvcam)
