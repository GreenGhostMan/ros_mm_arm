cmake_minimum_required(VERSION 2.8.3)
project(ros_mm_arm_developer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  genmsg
)

find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)

find_package(PCL REQUIRED)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   PickAndPlace.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
#link_directories(${catkin_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})


#______________________________GhostMan Add____________________________________#




#add_subdirectory(project/simple_pick_and_place)
#add_subdirectory(project/simple_hand_trainable)
#add_subdirectory(project/simple_object_recognize)


#PYTHON



#_______________________________End Ghost Add__________________________________#

add_executable(grasp_client src/myanmar_simple_grasp_client.cpp)
target_link_libraries(grasp_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS grasp_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(grasp_server src/myanmar_simple_grasp_server.cpp)
target_link_libraries(grasp_server ${catkin_LIBRARIES})
install(TARGETS grasp_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
