cmake_minimum_required(VERSION 2.8)
project(tabletop)

set(python_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/tabletop)

find_package(ecto REQUIRED)
find_package(ecto_pcl REQUIRED)
find_package(PCL REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

find_package(object_recognition_core REQUIRED)

find_package(genmsg REQUIRED)

if (genmsg_FOUND)
    find_package(ROS REQUIRED geometry_msgs sensor_msgs std_msgs)
    add_message_files(DIRECTORY msg FILES TabletopDetectionResult.msg Shape.msg Table.msg)
    generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)
    # generate ecto cells to wrap the different messages
    pubsub_gen_wrap(tabletop)
endif()






# temporary building of the ancient table finder
find_package(Eigen REQUIRED)
include_directories(SYSTEM
    ${Eigen_INCLUDE_DIRS}
    ${ecto_pcl_INCLUDE_DIRS}
)

find_library(tf_LIBRARIES tf /opt/ros/fuerte/stacks/geometry/tf/lib/)
find_library(rosconsole_LIBRARIES rosconsole /opt/ros/fuerte/lib/)
find_library(roscpp_LIBRARIES roscpp /opt/ros/fuerte/lib/)
find_library(roscpp_serialization_LIBRARIES roscpp_serialization /opt/ros/fuerte/lib/)
find_library(rostime_LIBRARIES rostime /opt/ros/fuerte/lib/)
find_library(pcl_ros_tf_LIBRARIES pcl_ros_tf /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/lib)
find_library(distance_field_LIBRARIES distance_field /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/lib/)

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

# add the tests
add_subdirectory(test)

# deal with some ROS stuff
install(FILES manifest.xml stack.yaml
    DESTINATION share/tabletop
)
install(DIRECTORY msg
    DESTINATION share/tabletop
    PATTERN ".svn" EXCLUDE
)
