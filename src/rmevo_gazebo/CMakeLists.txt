cmake_minimum_required(VERSION 2.8.3)
project(rmevo_gazebo)

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
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
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
  CATKIN_DEPENDS roscpp
)

include_directories(
${catkin_INCLUDE_DIRS}
${GAZEBO_INCLUDE_DIRS}
)

link_directories()

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(world_control SHARED plugins/world_control.cc)
target_link_libraries(world_control ${GAZEBO_LIBRARIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION lib/${PROJECT_NAME}
)

