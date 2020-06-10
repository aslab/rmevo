cmake_minimum_required(VERSION 2.8.3)
project(factory_ros)

## Uncomment if the package has a setup.py
catkin_python_setup()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  ImportModules.srv
  OutputString.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories()

link_directories(${GAZEBO_LIBRARY_DIRS})
catkin_install_python(PROGRAMS src/run_factory.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)