cmake_minimum_required(VERSION 3.9)
project(main)

find_package(cpprestsdk REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

include_directories(  ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable(main 
               restTest.cpp
               ./src/RosHandler.cpp)

target_link_libraries(main PRIVATE cpprestsdk::cpprest ${catkin_LIBRARIES})
