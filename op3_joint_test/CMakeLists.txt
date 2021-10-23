cmake_minimum_required(VERSION 3.0.2)
project(op3_joint_test)

find_package(catkin REQUIRED COMPONENTS
  # op3_manager
  robotis_controller_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    robotis_controller_msgs
    roscpp
    sensor_msgs
    std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_test
  src/joint_test.cpp
)

add_dependencies(joint_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(joint_test
  ${catkin_LIBRARIES}
)

install(TARGETS joint_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
