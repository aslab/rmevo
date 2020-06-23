cmake_minimum_required(VERSION 2.8.3)
project(rmevo)

## Uncomment if the package has a setup.py
catkin_python_setup()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  OutputString.srv
  RobotConfiguration.srv
  FitnessEvaluation.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS rospy roscpp
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories()

link_directories(${GAZEBO_LIBRARY_DIRS})
catkin_install_python(PROGRAMS src/run_rmevo.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
