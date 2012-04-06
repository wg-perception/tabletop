# create some messages for ROS

find_package(genmsg)
if (genmsg_FOUND)
  SET(TABLETOP_ROS_CELLS TableMsgAssembler)
endif()

find_package(object_recognition_core REQUIRED)
find_package(object_recognition_msgs REQUIRED)

find_pcl_package()
if (ROS_ELECTRIC_FOUND)
    include_directories(SYSTEM
                        /opt/ros/electric/stacks/arm_navigation_experimental/distance_field/include/
                        /opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/
                        /opt/ros/electric/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                        /opt/ros/electric/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include/
                        /opt/ros/electric/stacks/object_manipulation/household_objects_database/include/
                        /opt/ros/electric/stacks/sql_database/database_interface/include/)
    find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/electric/stacks/object_manipulation/household_objects_database/lib)
else()
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


include_directories(${CMAKE_SOURCE_DIR}/tabletop/include)

include_directories(SYSTEM
                    ${object_recognition_core_INCLUDE_DIRS}
                    ${object_recognition_msgs_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

add_definitions(-DPCL_VERSION_GE_160=1)

# deal with the detection TOD module
ectomodule(tabletop_table
  module.cpp
  Clusterer.cpp
  TableDetector.cpp
  TablePose.cpp
  tabletop_segmenter.cpp
  ${TABLETOP_ROS_CELLS}
)

link_ecto(tabletop_table
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${object_recognition_core_LIBRARIES}
    ${pcl_ros_tf_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_serialization_LIBRARIES}
    ${rostime_LIBRARIES}
    ${rosconsole_LIBRARIES}
    ${tf_LIBRARIES}
    marker_generator
)

install_ecto_module(tabletop_table)
