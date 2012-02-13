cmake_minimum_required(VERSION 2.8)
project(tabletop)

find_package(ecto REQUIRED)
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


rosbuild_add_library(marker_generator src/marker_generator.cpp )
add_dependencies(marker_generator tabletop_gencpp)

rosbuild_add_executable(tabletop_segmentation src/tabletop_segmentation.cpp )
target_link_libraries(tabletop_segmentation marker_generator)

find_library(tf_LIBRARIES tf /opt/ros/fuerte/stacks/geometry/tf/lib/)
find_library(rosconsole_LIBRARIES rosconsole /opt/ros/fuerte/lib/)
find_library(roscpp_LIBRARIES roscpp /opt/ros/fuerte/lib/)
find_library(roscpp_serialization_LIBRARIES roscpp_serialization /opt/ros/fuerte/lib/)
find_library(rostime_LIBRARIES rostime /opt/ros/fuerte/lib/)
find_library(pcl_ros_tf_LIBRARIES pcl_ros_tf /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/lib)
target_link_libraries(tabletop_segmentation
    ${PCL_LIBRARIES}
    ${pcl_ros_tf_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_serialization_LIBRARIES}
    ${rostime_LIBRARIES}
    ${rosconsole_LIBRARIES}
    ${tf_LIBRARIES}
)

add_subdirectory(src)


#rosbuild_add_library(tabletop_model_fitter src/model_fitter.cpp
#                                           src/iterative_distance_fitter.cpp)

#rosbuild_add_executable(tabletop_object_recognition src/tabletop_object_recognition.cpp
#                                                    src/marker_generator.cpp)
#target_link_libraries(tabletop_object_recognition tabletop_model_fitter)

#find_package(Boost REQUIRED)
#target_link_libraries(tabletop_object_recognition
#    ${Boost_LIBRARIES}
#)


#rosbuild_add_executable(segment_object_in_hand src/segment_object_in_hand.cpp)

#rosbuild_add_executable(ping_segment_object_in_hand src/ping_segment_object_in_hand.cpp)
