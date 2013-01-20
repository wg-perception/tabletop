cmake_minimum_required(VERSION 2.8)
project(object_recognition_tabletop)

find_package(catkin)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED ecto ecto_pcl ecto_ros ecto_ros genmsg moveit_core moveit_msgs object_recognition_core
                             object_recognition_msgs pcl_ros rosconsole roscpp shape_msgs sql_database tf)
catkin_package(INCLUDE_DIRS include)
add_definitions(-DROS_GROOVY_OR_ABOVE_FOUND=1)
add_definitions(-DPCL_VERSION_GE_160=1)

list(APPEND catkin_INCLUDE_DIRS /opt/ros/groovy/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
                                /opt/ros/groovy/stacks/object_manipulation/household_objects_database/include/
                                /opt/ros/groovy/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
)
find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/groovy/stacks/object_manipulation/household_objects_database/lib)
if (NOT ${household_objects_database_LIBRARIES} STREQUAL "household_objects_database_LIBRARIES-NOTFOUND")
list(APPEND catkin_LIBRARIES ${household_objects_database_LIBRARIES})
endif()
list(APPEND catkin_INCLUDE_DIRS /opt/ros/groovy/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
                                /opt/ros/groovy/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                                /opt/ros/groovy/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include/
                                /opt/ros/groovy/stacks/object_manipulation/household_objects_database/include/
)

else()
add_definitions(-DROS_GROOVY_OR_ABOVE_FOUND=0)
find_package(ecto REQUIRED)
find_package(ecto_ros REQUIRED)
find_package(genmsg REQUIRED)
find_package(object_recognition_core REQUIRED)
find_package(object_recognition_msgs REQUIRED)

find_package(catkin REQUIRED ecto ecto_pcl ecto_ros moveit_msgs object_recognition_core object_recognition_msgs tf)
catkin_stack()
catkin_project(object_recognition_tabletop
               INCLUDE_DIRS include
)

find_library(distance_field_LIBRARIES distance_field /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/lib/)
find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/lib)
find_library(pcl_ros_tf_LIBRARIES pcl_ros_tf /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/lib)
find_library(tf_LIBRARIES tf /opt/ros/fuerte/stacks/geometry/tf/lib/)
find_library(rosconsole_LIBRARIES rosconsole /opt/ros/fuerte/lib/)
find_library(roscpp_LIBRARIES roscpp /opt/ros/fuerte/lib/)
find_library(roscpp_serialization_LIBRARIES roscpp_serialization /opt/ros/fuerte/lib/)
find_library(rostime_LIBRARIES rostime /opt/ros/fuerte/lib/)

list(APPEND catkin_INCLUDE_DIRS ${object_recognition_core_INCLUDE_DIRS}
                                ${object_recognition_msgs_INCLUDE_DIRS}
                                /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
                                /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/include/
                                /opt/ros/fuerte/stacks/bullet/include/
                                /opt/ros/fuerte/stacks/geometry/tf/include
                                /opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include/
                                /opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include/
                                /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/include/
                                /opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                                /opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include/
                                /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/include/
                                /opt/ros/fuerte/stacks/sql_database/database_interface/include/
)

list(APPEND catkin_LIBRARIES ${distance_field_LIBRARIES}
                             ${household_objects_database_LIBRARIES}
                             ${pcl_ros_tf_LIBRARIES}
                             ${tf_LIBRARIES}
                             ${rosconsole_LIBRARIES}
                             ${roscpp_LIBRARIES}
                             ${roscpp_serialization_LIBRARIES}
                             ${rostime_LIBRARIES}
)
endif()

# deal with Python
catkin_python_setup()

add_subdirectory(python)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE 1)

# include some directories
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}
                    include
)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)

if (ROS_FUERTE_FOUND)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the lib directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()
endif()

add_subdirectory(src)

# add the tests
add_subdirectory(test)

catkin_doxygen(object_recognition_tabletop-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)
