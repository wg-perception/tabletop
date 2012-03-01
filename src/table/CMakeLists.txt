# create some messages for ROS

find_package(genmsg)
if (genmsg_FOUND)
  SET(TABLETOP_ROS_CELLS TableMsgAssembler)
endif()

if (ROS_ELECTRIC_FOUND)
  find_ros_package(geometry_msgs)
  find_ros_package(sensor_msgs)
  find_ros_package(visualization_msgs)
  include_directories(${ecto_ros_INCLUDE_DIRS})
else()
find_package(ROS REQUIRED geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
endif()

find_package(object_recognition_core REQUIRED)
include_directories(SYSTEM
    ${object_recognition_core_INCLUDE_DIRS}
)

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
