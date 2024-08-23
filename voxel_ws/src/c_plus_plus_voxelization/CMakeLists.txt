cmake_minimum_required(VERSION 3.10)
project(c_plus_plus_voxelization)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen3 package
find_package(Eigen3 REQUIRED)

# Find OpenCV package
find_package(OpenCV REQUIRED)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
endif()

# Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  message_generation  # Add this to include message generation capabilities
)

# Add message files
add_message_files(
  FILES
  weight.msg
  raw_loadcell.msg
)

# Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs  # Add dependencies for your messages here
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rosbag tf sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_ros cv_bridge
)

# Add the executable
add_executable(voxelization src/voxelization.cpp src/main.cpp src/bag_data_reader.cpp src/process_voxelization.cpp src/transform_adjuster.cpp)

# Link Eigen3, OpenCV, and catkin libraries
target_link_libraries(voxelization ${EIGEN3_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})