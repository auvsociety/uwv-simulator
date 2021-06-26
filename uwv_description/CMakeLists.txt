cmake_minimum_required(VERSION 2.8.3)
project(uwv_description)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
              gazebo_ros
              roscpp
              std_msgs
              sensor_msgs
              geometry_msgs)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_subdirectory(src/plugin_depth_sensor)