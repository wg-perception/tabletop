cmake_minimum_required(VERSION 2.8)
project(object_recognition_tabletop)
find_package(catkin REQUIRED)
catkin_stack()
catkin_project(object_recognition_tabletop
               INCLUDE_DIRS include
               DEPENDS OpenCV object_recognition_core object_recognition_msgs
)

# deal with Python
catkin_python_setup()

find_package(ecto REQUIRED)

add_subdirectory(python)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE 1)

find_package(ecto_pcl REQUIRED)
find_pcl_package()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(include)
find_package(object_recognition_core REQUIRED)


if (NOT ROS_GROOVY_FOUND)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the lib directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()
endif()

# deal with the messages
find_package(genmsg REQUIRED)

# temporary building of the ancient table finder
find_package(Eigen REQUIRED)
include_directories(SYSTEM
                    ${Eigen_INCLUDE_DIRS}
                    ${ecto_pcl_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

if (ROS_ELECTRIC_FOUND)
    find_ros_package(tf)
    find_ros_package(distance_field)
    find_ros_package(household_objects_database)
endif()
if (ROS_FUERTE_FOUND)
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
endif()
if (ROS_GROOVY_FOUND)
#find_package(catkin REQUIRED distance_field pcl_ros_tf rosconsole roscpp rostime tf)
find_package(catkin REQUIRED rosconsole roscpp rostime)
   find_library(tf_LIBRARIES tf /opt/ros/fuerte/stacks/geometry/tf/lib/)
    find_library(roscpp_serialization_LIBRARIES roscpp_serialization /opt/ros/fuerte/lib/)
    find_library(rostime_LIBRARIES rostime /opt/ros/fuerte/lib/)
    find_library(pcl_ros_tf_LIBRARIES pcl_ros_tf /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/lib)
    find_library(distance_field_LIBRARIES distance_field /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/lib/)

    include_directories(/opt/ros/fuerte/stacks/geometry/tf/include)
    include_directories(/opt/ros/fuerte/stacks/bullet/include/)
    include_directories(/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include/)
    include_directories(/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include/)
    include_directories(/opt/ros/fuerte/stacks/perception_pcl/pcl_ros/include/)
include_directories(SYSTEM ${rosconsole_INCLUDE_DIRS}
                           ${roscpp_INCLUDE_DIRS}
)
endif()

add_subdirectory(src)

#find_package(ROS REQUIRED geometry)
#include_directories(${geometry_INCLUDE_DIRS})

# add the tests
add_subdirectory(test)


catkin_doxygen(object_recognition_tabletop-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)
