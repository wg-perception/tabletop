cmake_minimum_required(VERSION 2.8)
project(tabletop)

find_package(ecto REQUIRED)
find_package(ecto_pcl REQUIRED)
find_package(PCL REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})


find_package(genmsg REQUIRED)

if (genmsg_FOUND)
  find_package(ROS REQUIRED geometry_msgs sensor_msgs std_msgs)
  add_message_files(DIRECTORY msg FILES TabletopDetectionResult.msg Shape.msg Table.msg)
  generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)
endif()



# temporary building of the ancient table finder

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${ecto_pcl_INCLUDE_DIRS})

include_directories(/opt/ros/fuerte/stacks/geometry/tf/include)
include_directories(/opt/ros/fuerte/stacks/bullet/include/)
include_directories(/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include/)
include_directories(/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include/)
include_directories(/opt/ros/fuerte/stacks/perception_pcl/pcl_ros/include/)
#find_package(ROS REQUIRED geometry)
#include_directories(${geometry_INCLUDE_DIRS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src)

#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/
  DESTINATION ${PYTHON_PACKAGES_PATH}
  COMPONENT main
)

catkin_add_env_hooks(10tabletop SHELLS all)
