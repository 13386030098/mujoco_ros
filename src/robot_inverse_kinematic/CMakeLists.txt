cmake_minimum_required(VERSION 2.8.3)
project(robot_inverse_kinematic)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR};/usr/include/eigen3/Eigen/Eigen")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES davinci_planing
  CATKIN_DEPENDS roscpp rospy std_msgs 
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

# davinci_kdl 
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(robot_inverse_single src/robot_inverse_single.cpp)
target_link_libraries(robot_inverse_single ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_inverse_dual src/robot_inverse_dual.cpp)
target_link_libraries(robot_inverse_dual ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_inverse_single_ver1 src/robot_inverse_single_ver1.cpp)
target_link_libraries(robot_inverse_single_ver1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})




