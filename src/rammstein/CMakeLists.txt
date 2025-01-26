cmake_minimum_required(VERSION 3.8)
project(rammstein)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  ff_msgs
  tf2_ros
  tf2
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Source files
set(LEADER_SOURCE
  src/leader_simple.cpp
)

set(FOLLOWER_SOURCE
  src/follower_simple.cpp
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs 
  DEPENDS Eigen3
)

# Executables
add_executable(rammstein_leader ${LEADER_SOURCE})
add_executable(rammstein_follower ${FOLLOWER_SOURCE})

# Link libraries for leader
target_link_libraries(rammstein_leader
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# Link libraries for follower
target_link_libraries(rammstein_follower
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# Install
install(TARGETS rammstein_leader rammstein_follower
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ##FOR RUBEN
  #  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)


