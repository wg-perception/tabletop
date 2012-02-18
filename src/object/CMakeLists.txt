add_library(marker_generator SHARED marker_generator.cpp)
add_dependencies(marker_generator tabletop_gencpp)

# old stuff, useless now
#rosbuild_add_executable(tabletop_segmentation tabletop_segmentation.cpp )
#target_link_libraries(tabletop_segmentation marker_generator)

#target_link_libraries(tabletop_segmentation
#    ${PCL_LIBRARIES}
#    ${pcl_ros_tf_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${roscpp_serialization_LIBRARIES}
#    ${rostime_LIBRARIES}
#    ${rosconsole_LIBRARIES}
#    ${tf_LIBRARIES}
#)

include_directories(/opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/include/)
include_directories(/opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/)
include_directories(/opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/)
include_directories(/opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include/)
include_directories(/opt/ros/fuerte/stacks/object_manipulation/household_objects_database/include/)
include_directories(/opt/ros/fuerte/stacks/sql_database/database_interface/include/)

add_library(tabletop_model_fitter SHARED
    model_fitter.cpp
    iterative_distance_fitter.cpp
    tabletop_object_recognition.cpp
)

find_package(Boost REQUIRED)
target_link_libraries(tabletop_model_fitter
    marker_generator
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    object_recognition_common
    ${pcl_ros_tf_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_serialization_LIBRARIES}
    ${rostime_LIBRARIES}
    ${rosconsole_LIBRARIES}
    ${tf_LIBRARIES}
    marker_generator
    ${distance_field_LIBRARIES}
)

# Create a library for tabletop db interfaces
add_library(tabletop_db SHARED
    db_sql_household.cpp
    db_mesh.cpp
)

find_package(object_recognition REQUIRED)
include_directories(${object_recognition_INCLUDE_DIRS})
include_directories(/opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/)
target_link_libraries(tabletop_db
    object_recognition_db
)

install(TARGETS tabletop_db
    DESTINATION lib
)



# old stuff, useless now
#rosbuild_add_executable(tabletop_object_recognition tabletop_object_recognition.cpp)

# deal with the detection module
find_package(object_recognition REQUIRED)
include_directories(${object_recognition_INCLUDE_DIRS})

ectomodule(tabletop_object
    module.cpp
    ObjectRecognizer.cpp
)

link_ecto(tabletop_object
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    object_recognition_common
    ${pcl_ros_tf_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_serialization_LIBRARIES}
    ${rostime_LIBRARIES}
    ${rosconsole_LIBRARIES}
    ${tf_LIBRARIES}
    marker_generator
    tabletop_model_fitter
    tabletop_db
)

install_ecto_module(tabletop_object)
