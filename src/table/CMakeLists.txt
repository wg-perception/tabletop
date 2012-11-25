# create some messages for ROS

find_package(genmsg)
if (genmsg_FOUND)
  SET(TABLETOP_ROS_CELLS TableMsgAssembler.cpp TableVisualizationMsgAssembler.cpp)
endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(ecto)

if (ROS_FUERTE_FOUND)
find_package(object_recognition_core REQUIRED)
find_package(object_recognition_msgs REQUIRED)
    include_directories(SYSTEM
                        /opt/ros/fuerte/stacks/arm_navigation_experimental/distance_field/include/
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
    find_library(tf_LIBRARIES tf /opt/ros/fuerte/stacks/geometry/tf/lib/)
else()
    include_directories(SYSTEM
                        /opt/ros/groovy/stacks/object_manipulation/household_objects_database_msgs/msg_gen/cpp/include/
                        /opt/ros/goovy/stacks/object_manipulation/household_objects_database_msgs/srv_gen/cpp/include/
                        /opt/ros/groovy/stacks/object_manipulation/household_objects_database/include/
                        /opt/ros/groovy/stacks/sql_database/database_interface/include/
    )
    find_library(household_objects_database_LIBRARIES household_objects_database /opt/ros/groovy/stacks/object_manipulation/household_objects_database/lib)
endif()


include_directories(${CMAKE_SOURCE_DIR}/tabletop/include)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                    ${object_recognition_core_INCLUDE_DIRS}
                    ${object_recognition_msgs_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

# deal with the detection TOD module
ectomodule(tabletop_table DESTINATION ${PROJECT_NAME}/ecto_cells
                          INSTALL
  Clusterer.cpp
  module.cpp
  TableDetector.cpp
  TablePose.cpp
  tabletop_segmenter.cpp
  ${TABLETOP_ROS_CELLS}
)

link_ecto(tabletop_table
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${object_recognition_core_LIBRARIES}
    ${OpenCV_LIBS}
    ${OpenCV_LIBRARIES}
    ${pcl_ros_tf_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_serialization_LIBRARIES}
    ${rostime_LIBRARIES}
    ${rosconsole_LIBRARIES}
    ${tf_LIBRARIES}
    marker_generator
)
