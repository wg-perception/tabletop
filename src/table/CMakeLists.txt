# create some messages for ROS
find_package(genmsg)
if (genmsg_FOUND)
#TODO
  #SET(TABLETOP_ROS_CELLS TablePublisher)
endif()

find_package(object_recognition REQUIRED)
include_directories(${object_recognition_INCLUDE_DIRS})

# deal with the detection TOD module
ectomodule(tabletop_table
  module.cpp
  Clusterer.cpp
  TableDetector.cpp
  TablePose.cpp
  ${TABLETOP_ROS_CELLS}
)

link_ecto(tabletop_table
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  object_recognition_common
)

install_ecto_module(tabletop_table)
