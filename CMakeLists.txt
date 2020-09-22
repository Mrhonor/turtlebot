cmake_minimum_required(VERSION 2.8.3)
project(aruco_listener)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    OpenCV
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
)



