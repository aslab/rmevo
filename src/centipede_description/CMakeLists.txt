cmake_minimum_required(VERSION 2.8.3)
project(centipede_description)

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
  xacro
  urdf
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

include_directories(
${catkin_INCLUDE_DIRS}
${GAZEBO_INCLUDE_DIRS}
)

link_directories()

link_directories(${GAZEBO_LIBRARY_DIRS})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION lib/${PROJECT_NAME}
)

# Generate .urdf files from .urdf.xacro files
find_package(xacro REQUIRED)

# Xacro files
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.urdf.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})

  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(${it} ${output_filename})

  list(APPEND urdf_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(media_files ALL DEPENDS ${urdf_files})

