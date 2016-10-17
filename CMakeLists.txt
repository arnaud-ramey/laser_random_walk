### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(laser_random_walk)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
find_package(catkin REQUIRED COMPONENTS
  roscpp  nav_msgs  tf)
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
### 5 - Invoke message/service/action generation (generate_messages())
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp  nav_msgs  tf
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(laser_random_walk.exe src/laser_random_walk.cpp)
target_link_libraries(laser_random_walk.exe ${catkin_LIBRARIES})

add_executable(odom2tf.exe src/odom2tf.cpp)
target_link_libraries(odom2tf.exe ${catkin_LIBRARIES})

add_executable(wall_follower.exe src/wall_follower.cpp)
target_link_libraries(wall_follower.exe ${catkin_LIBRARIES})

### 8 - Tests to build (catkin_add_gtest())
#ADD_SUBDIRECTORY(test)
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
