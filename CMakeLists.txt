cmake_minimum_required(VERSION 2.8.3)
project(aruco_listener)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    OpenCV
    Eigen3
)

add_message_files(
    FILES
)
  
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        roscpp
    	geometry_msgs
        sensor_msgs
        cv_bridge
        message_runtime
    DEPENDS
        ${LIBS}
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

add_executable(aruco_listener
    src/aruco_main.cpp
)

target_link_libraries(aruco_listener 
    ${catkin_LIBRARIES}
	Eigen3::Eigen
)



