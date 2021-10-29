cmake_minimum_required(VERSION 3.0.2)
project(op3_walk_test)

find_package(catkin REQUIRED COMPONENTS
  # op3_manager
  robotis_controller_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    robotis_controller_msgs
    sensor_msgs
    std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(walk_test
  src/walk_test.cpp
)

add_dependencies(walk_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(walk_test
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(TARGETS walk_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
