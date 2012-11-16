include_directories(SYSTEM /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/)

add_library(marker_generator SHARED marker_generator.cpp)
add_dependencies(marker_generator tabletop_gencpp)

install(TARGETS marker_generator
        DESTINATION lib
)

find_package(ecto REQUIRED)
find_package(PCL REQUIRED)
if (ROS_FUERTE_FOUND)
    include_directories(SYSTEM
                        /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/include/
                        /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/bullet/include/
                        /opt/ros/fuerte/stacks/geometry/tf/include/
                        /opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/include/
                        /opt/ros/fuerte/stacks/sql_database/database_interface/include/
    )
    find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/lib)
endif()
if (ROS_GROOVY_FOUND)
find_package(catkin REQUIRED moveit_core pcl_ros rosconsole roscpp tf)
include_directories(SYSTEM ${distance_field_INCLUDE_DIRS}
                        /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/include/
                        /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include/
                        /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/include/
                        /opt/ros/fuerte/stacks/sql_database/database_interface/include/
    )
    find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/fuerte/stacks/object_manipulation/household_objects_database/lib)
    find_library(distance_field_LIBRARIES distance_field /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/lib/)
endif()

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

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
    ${object_recognition_core_LIBRARIES}
    ${pcl_ros_tf_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_serialization_LIBRARIES}
    ${rostime_LIBRARIES}
    ${rosconsole_LIBRARIES}
    ${tf_LIBRARIES}
    marker_generator
    ${distance_field_LIBRARIES}
)

install(TARGETS tabletop_model_fitter
        DESTINATION lib
)

# old stuff, useless now
#rosbuild_add_executable(tabletop_object_recognition tabletop_object_recognition.cpp)

# deal with the detection module
find_package(object_recognition_core REQUIRED)
include_directories(SYSTEM
    ${object_recognition_core_INCLUDE_DIRS}
)

ectomodule(tabletop_object DESTINATION ${PROJECT_NAME}/ecto_cells
                           INSTALL
    module.cpp
    ObjectRecognizer.cpp
)

link_ecto(tabletop_object
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${object_recognition_core_LIBRARIES}
    ${pcl_ros_tf_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_serialization_LIBRARIES}
    ${rostime_LIBRARIES}
    ${rosconsole_LIBRARIES}
    ${household_objects_database_LIBRARIES}
    ${tf_LIBRARIES}
    marker_generator
    tabletop_model_fitter
    tabletop_db
)
