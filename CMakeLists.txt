cmake_minimum_required(VERSION 2.8.3)
project(ceng_goto)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ceng_goto
 CATKIN_DEPENDS roscpp rospy
 DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_executable(goto_xy src/goto.cpp)
target_include_directories(goto_xy PRIVATE ${INCLUDE_DIRS})
target_link_libraries(goto_xy
  ${catkin_LIBRARIES}
)
