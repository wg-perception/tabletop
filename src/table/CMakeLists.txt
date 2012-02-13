# create some messages for ROS
find_package(genmsg)
if (genmsg_FOUND)
#TODO
  #SET(TABLETOP_ROS_CELLS TablePublisher)
endif()

# deal with the detection TOD module
ectomodule(tabletop_table
  module.cpp
  Clusterer.cpp
  ConvexHull.cpp
  TableDetector.cpp
  ${TABLETOP_ROS_CELLS}
)

link_ecto(tabletop_table
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

install_ecto_module(tabletop_table)
