cmake_minimum_required(VERSION 2.8)
project(object_recognition_tabletop)

find_package(catkin)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED ecto ecto_pcl ecto_ros genmsg moveit_msgs object_recognition_core
                             object_recognition_msgs opencv_candidate rosconsole roscpp shape_msgs)
catkin_package(INCLUDE_DIRS include)
add_definitions(-DROS_GROOVY_OR_ABOVE_FOUND=1)
add_definitions(-DPCL_VERSION_GE_160=1)

find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/groovy/stacks/object_manipulation/household_objects_database/lib)
if (NOT ${household_objects_database_LIBRARIES} STREQUAL "household_objects_database_LIBRARIES-NOTFOUND")
find_package(moveit_core)
find_package(pcl_ros)
find_package(sql_database)
find_package(tf)
list(APPEND catkin_LIBRARIES ${household_objects_database_LIBRARIES}
                             ${moveit_core_LIBRARIES}
                             ${pcl_ros_LIBRARIES}
                             ${sql_database_LIBRARIES}
                             ${tf_LIBRARIES}
)
list(APPEND catkin_INCLUDE_DIRS
                                /opt/ros/groovy/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                                /opt/ros/groovy/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include/
                                /opt/ros/groovy/stacks/object_manipulation/household_objects_database/include/
)
endif()

else()
add_definitions(-DROS_GROOVY_OR_ABOVE_FOUND=0)
find_package(ROS REQUIRED ecto ecto_pcl ecto_ros genmsg moveit_msgs object_recognition_core
                          object_recognition_msgs opencv_candidate rosconsole roscpp)
catkin_stack()
catkin_project(object_recognition_tabletop
               INCLUDE_DIRS include
)

set(catkin_LIBRARIES ${ROS_LIBRARIES})
set(catkin_INCLUDE_DIRS ${ROS_INCLUDE_DIRS})
set(CATKIN_DEVEL_PREFIX ${CMAKE_BINARY_DIR})
set(CATKIN_PACKAGE_BIN_DESTINATION share/${PROJECT_NAME}/bin)
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include/${PROJECT_NAME})
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_PYTHON_BUILD_DESTINATION gen/py/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_PYTHON_DESTINATION ${INSTALLED_PYTHONPATH}/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/lib)
if (NOT ${household_objects_database_LIBRARIES} STREQUAL "household_objects_database_LIBRARIES-NOTFOUND")
find_library(distance_field_LIBRARIES distance_field /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/lib/)
find_library(pcl_ros_tf_LIBRARIES pcl_ros_tf /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/lib)
find_library(tf_LIBRARIES tf /opt/ros/fuerte/stacks/geometry/tf/lib/)

list(APPEND catkin_INCLUDE_DIRS /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
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
)
endif()
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
endif()

add_subdirectory(src)

# add the tests
add_subdirectory(test)

# add the docs
catkin_doxygen(object_recognition_tabletop-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)

# install the configuration files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/conf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
