cmake_minimum_required(VERSION 2.8.3)
project(omega7_drive)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
)

catkin_package(
#  INCLUDE_DIRS include

)


include_directories(
  "include/omega7_driver"
  ${catkin_INCLUDE_DIRS}
  "/home/zzz/davinci/src/omega7_drive/include/omega7_drive"
)
add_executable(read_pose src/read_pose.cpp)
target_link_libraries(read_pose  ${catkin_LIBRARIES} ${Boost_LIBRARIES} "/usr/lib/libdhd.so.3" "/usr/lib/libdrd.so.3")

#add_executable(read_pose_multi src/read_pose_multi.cpp)
#target_link_libraries(read_pose_multi  ${catkin_LIBRARIES} ${Boost_LIBRARIES} "/home/zzz/davinci/src/omega7_drive/include/omega7_drive/libdhd.so.3.7.3" "/#home/zzz/davinci/src/omega7_drive/include/omega7_drive/libdrd.so.3.7.3"	)
