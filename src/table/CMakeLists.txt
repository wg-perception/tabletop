# create some messages for ROS
find_package(genmsg)
if (genmsg_FOUND)
  SET(TABLETOP_ROS_CELLS TableMsgAssembler)
endif()

find_package(object_recognition REQUIRED)
include_directories(${object_recognition_INCLUDE_DIRS})

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
  object_recognition_common
    ${pcl_ros_tf_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${roscpp_serialization_LIBRARIES}
    ${rostime_LIBRARIES}
    ${rosconsole_LIBRARIES}
    ${tf_LIBRARIES}
    marker_generator
)

install_ecto_module(tabletop_table)
