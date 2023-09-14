cmake_minimum_required(VERSION 3.9)
project(RosRestBridge)

find_package(cpprestsdk REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

include_directories(  ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable(${PROJECT_NAME}
               ./src/main.cpp
               ./src/BridgeManager.cpp
               ./src/RestHandler.cpp
               ./src/RosHandler.cpp
        )

target_link_libraries(${PROJECT_NAME} PRIVATE cpprestsdk::cpprest ${catkin_LIBRARIES})
