cmake_minimum_required(VERSION 2.8.3)
project(aruco_listener)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
)

add_message_files(
    FILES
)
  
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
    CATKIN_DEPENDS
        roscpp
	geometry_msgs
        message_runtime
    DEPENDS
        ${LIBS}
)

include_directories( 
    ${catkin_INCLUDE_DIRS}
)

add_executable(aruco_listener
    src/aruco_listener.cpp
)

target_link_libraries(aruco_listener 
    ${catkin_LIBRARIES}
)



