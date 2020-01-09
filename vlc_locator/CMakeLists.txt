cmake_minimum_required(VERSION 2.8.3)
project(vlc_locator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

find_package(OpenCV)

catkin_package()


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#find_package(OpenCV REQUIRED)
add_executable(locator 
  src/publisher.cpp
  src/imageProcess.cpp
  src/positioningCalculation.cpp
)
target_link_libraries(locator ${catkin_LIBRARIES}  ${OpenCV_LIBS})

#find_package(OpenCV REQUIRED)
add_executable(locator_2 
  src/publisher_2.cpp
  src/imageProcess.cpp
  src/positioningCalculation.cpp
)
target_link_libraries(locator_2 ${catkin_LIBRARIES}  ${OpenCV_LIBS})

#find_package(OpenCV REQUIRED)
add_executable(locator_3
  src/publisher_3.cpp
  src/imageProcess.cpp
  src/positioningCalculation.cpp
)
target_link_libraries(locator_3 ${catkin_LIBRARIES}  ${OpenCV_LIBS})

#find_package(OpenCV REQUIRED)
add_executable(locator_2_max
  src/publisher_2_max.cpp
  src/imageProcess.cpp
  src/positioningCalculation.cpp
)
target_link_libraries(locator_2_max ${catkin_LIBRARIES}  ${OpenCV_LIBS})

#find_package(OpenCV REQUIRED)
add_executable(test
  src/test.cpp
  src/imageProcess.cpp
  src/positioningCalculation.cpp
)
target_link_libraries(test ${catkin_LIBRARIES}  ${OpenCV_LIBS})



add_definitions(-std=c++17)


