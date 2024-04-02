cmake_minimum_required(VERSION 3.0.2)
project(lab_gazebo_models)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roslaunch
)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_ros roslaunch
  DEPENDS gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(hinge_control SHARED src/hinge_control.cpp)
target_link_libraries(hinge_control ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
