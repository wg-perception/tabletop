rosbuild_add_library(marker_generator marker_generator.cpp)
add_dependencies(marker_generator tabletop_gencpp)

rosbuild_add_executable(tabletop_segmentation tabletop_segmentation.cpp )
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

#rosbuild_add_library(tabletop_model_fitter model_fitter.cpp
#                                           iterative_distance_fitter.cpp)

#rosbuild_add_executable(tabletop_object_recognition tabletop_object_recognition.cpp
#                                                    marker_generator.cpp)
#target_link_libraries(tabletop_object_recognition tabletop_model_fitter)

#find_package(Boost REQUIRED)
#target_link_libraries(tabletop_object_recognition
#    ${Boost_LIBRARIES}
#)


#rosbuild_add_executable(ping_segment_object_in_hand src/ping_segment_object_in_hand.cpp)
