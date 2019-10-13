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
<<<<<<< HEAD
add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(2333 src/2333.cpp)
target_link_libraries(2333 ${catkin_LIBRARIES}  ${OpenCV_LIBS})

# add_executable(image_listener src/image_listener.cpp)
# target_link_libraries(image_listener ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(publisher_2 src/publisher_2.cpp)
target_link_libraries(publisher_2 ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(publisher_2_max src/publisher_2_max.cpp)
target_link_libraries(publisher_2_max ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(publisher_2_2 src/publisher_2_2.cpp)
target_link_libraries(publisher_2_2 ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(publisher_2_3 src/publisher_2_3.cpp)
target_link_libraries(publisher_2_3 ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(publisher_3 src/publisher_3.cpp)
target_link_libraries(publisher_3 ${catkin_LIBRARIES}  ${OpenCV_LIBS})
=======
add_executable(locator 
  src/publisher.cpp
  src/imageProcess.cpp
  src/positioningCalculation.cpp
)
target_link_libraries(locator ${catkin_LIBRARIES}  ${OpenCV_LIBS})
>>>>>>> beta

# add_executable(locator_2
#   src/publisher_2.cpp
# )
# target_link_libraries(locator ${catkin_LIBRARIES}  ${OpenCV_LIBS})


<<<<<<< HEAD
add_executable(publisher_test src/publisher_test.cpp)
target_link_libraries(publisher_test ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(LED_ID_test src/LED_ID_test.cpp)
target_link_libraries(LED_ID_test ${catkin_LIBRARIES}  ${OpenCV_LIBS})
=======
>>>>>>> beta

add_definitions(-std=c++11)


