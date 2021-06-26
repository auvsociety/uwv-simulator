cmake_minimum_required(VERSION 2.8.3)
project(uwv_env)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

include_directories(${catkin_INCLUDE_DIRS})
