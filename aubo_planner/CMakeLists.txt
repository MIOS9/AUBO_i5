cmake_minimum_required(VERSION 2.8.3)
project(aubo_planner)

find_package(catkin REQUIRED COMPONENTS 
  aubo_msgs  
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  pluginlib 
  cmake_modules
  roscpp 
  std_msgs 
  sensor_msgs
  control_msgs 
  trajectory_msgs 
  actionlib_msgs 
  actionlib
  urdf 
  industrial_msgs 
  industrial_utils
)

find_package(Boost REQUIRED system filesystem date_time thread)

# 处理不同ROS版本的Eigen库查找
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "noetic")
    find_package(Eigen3 REQUIRED)
    set(EIGEN3_INCLUDE_DIRS "/usr/include/eigen3")  # 明确指定Eigen3的路径
    message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Eigen3 Package in aubo_planner for kinetic or noetic")
elseif("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    find_package(Eigen REQUIRED)
    set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")  # 明确指定Eigen的路径
    message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Eigen Package in aubo_planner for indigo")
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    control_msgs
    actionlib
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    trajectory_msgs
    industrial_msgs
    actionlib
    actionlib_msgs
    industrial_utils
    urdf
    LIBRARIES ${PROJECT_NAME}_dummy
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM ${Boost_INCLUDE_DIR})

# 统一包含Eigen头文件目录
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "noetic")
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
    message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Using EIGEN3_INCLUDE_DIRS for kinetic or noetic")
elseif("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
    message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Using EIGEN_INCLUDE_DIRS for indigo")
endif()

link_directories(${catkin_LIBRARY_DIRS})

add_executable(aubo_planner_test src/aubo_planner_test.cpp)
target_link_libraries(aubo_planner_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(aubo_ros_plan src/aubo_ros_plan.cpp)
#target_link_libraries(aubo_ros_plan ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS aubo_ros_plan DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS script/aubo_planner/aubo_ros_plan
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

